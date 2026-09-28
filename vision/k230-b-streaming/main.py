import gc
import network
import os
import sys
import time
import uctypes
import _thread

import multimedia as mm
from media.media import *
from media.sensor import *
from media.vencoder import *


WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
EXPECTED_GATEWAY = "192.168.1.1"

WIFI_CONNECT_TIMEOUT_MS = 30000
RECONNECT_DELAY_MS = 3000

RTSP_SESSION_NAME = "video"
RTSP_PORT = 8554
VIDEO_WIDTH = 512
VIDEO_HEIGHT = 288
VIDEO_BIT_RATE_KBPS = 200
VIDEO_FRAME_RATE = 30
VIDEO_GOP_LENGTH = 30
VENC_STREAM_TIMEOUT_MS = 1000
RTSP_SEND_TIMESTAMP = 1000
STREAM_WARNING_INTERVAL = 30
IDR_REQUEST_INTERVAL_FRAMES = 30
STREAM_THREAD_STOP_TIMEOUT_MS = 3000


def _exitpoint():
    if hasattr(os, "exitpoint"):
        os.exitpoint()


def _sleep_ms(duration_ms):
    remaining_ms = duration_ms
    while remaining_ms > 0:
        _exitpoint()
        step_ms = 100 if remaining_ms > 100 else remaining_ms
        time.sleep_ms(step_ms)
        remaining_ms -= step_ms


def _print_exception(prefix, error):
    print(prefix, error)
    if hasattr(sys, "print_exception"):
        sys.print_exception(error)


def connect_wifi():
    sta = network.WLAN(network.STA_IF)

    try:
        if sta.isconnected():
            sta.disconnect()
    except Exception:
        pass

    try:
        sta.active(False)
        _sleep_ms(300)
    except Exception:
        pass

    sta.active(True)
    _sleep_ms(500)

    print("[WIFI] Connecting to", WIFI_SSID)
    sta.connect(WIFI_SSID, WIFI_PASSWORD)
    started_ms = time.ticks_ms()

    while not sta.isconnected():
        _exitpoint()
        if time.ticks_diff(time.ticks_ms(), started_ms) >= WIFI_CONNECT_TIMEOUT_MS:
            raise RuntimeError("Wi-Fi DHCP connection timed out")
        time.sleep_ms(250)

    config = sta.ifconfig()
    print("[WIFI] DHCP IP:", config[0])
    print("[WIFI] Netmask:", config[1])
    print("[WIFI] Gateway:", config[2])
    print("[WIFI] DNS:", config[3])

    if config[2] != EXPECTED_GATEWAY:
        print("[WIFI] Warning: expected gateway", EXPECTED_GATEWAY)

    return sta, config


class RtspServer:
    def __init__(self):
        self.rtspserver = mm.rtsp_server()
        self.venc_chn = VENC_CHN_ID_0

        self.sensor = None
        self.encoder = None
        self.link = None

        self.start_stream = False
        self.runthread_over = True
        self.thread_error = None
        self._thread_started = False

        self._media_initialized = False
        self._encoder_created = False
        self._encoder_started = False
        self._sensor_started = False
        self._rtsp_initialized = False
        self._session_created = False
        self._rtsp_started = False
        self._rtsp_destroyed = False

    @staticmethod
    def _require_success(operation, result):
        if result is not None and result != 0:
            raise RuntimeError("%s failed: %s" % (operation, result))

    @staticmethod
    def _safe_cleanup(operation, callback):
        try:
            callback()
        except Exception as error:
            _print_exception("[CLEANUP] %s:" % operation, error)

    @staticmethod
    def _print_repeated_warning(prefix, count, detail):
        if count == 1 or count % STREAM_WARNING_INTERVAL == 0:
            print("%s %d times: %s" % (prefix, count, detail))

    def _init_media(self):
        width = ALIGN_UP(VIDEO_WIDTH, 16)

        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(
            width=width,
            height=VIDEO_HEIGHT,
            alignment=12,
        )
        self.sensor.set_pixformat(Sensor.YUV420SP)

        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            self.venc_chn,
            8,
            width,
            VIDEO_HEIGHT,
        )

        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn),
        )
        MediaManager.init()
        self._media_initialized = True

        channel_attributes = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            width,
            VIDEO_HEIGHT,
            bit_rate=VIDEO_BIT_RATE_KBPS,
            gopLen=VIDEO_GOP_LENGTH,
            src_frame_rate=VIDEO_FRAME_RATE,
            dst_frame_rate=VIDEO_FRAME_RATE,
        )
        self._require_success(
            "Encoder.Create",
            self.encoder.Create(self.venc_chn, channel_attributes),
        )
        self._encoder_created = True

    def start(self):
        try:
            self._init_media()

            self._require_success(
                "rtspserver_init",
                self.rtspserver.rtspserver_init(RTSP_PORT),
            )
            self._rtsp_initialized = True

            self._require_success(
                "rtspserver_createsession",
                self.rtspserver.rtspserver_createsession(
                    RTSP_SESSION_NAME,
                    mm.multi_media_type.media_h264,
                    False,
                ),
            )
            self._session_created = True

            self._require_success(
                "rtspserver_start",
                self.rtspserver.rtspserver_start(),
            )
            self._rtsp_started = True

            self._require_success(
                "Encoder.Start",
                self.encoder.Start(self.venc_chn),
            )
            self._encoder_started = True

            self._require_success("Sensor.run", self.sensor.run())
            self._sensor_started = True

            self.thread_error = None
            self.start_stream = True
            self.runthread_over = False
            _thread.start_new_thread(self._do_rtsp_stream, ())
            self._thread_started = True
        except Exception:
            if not self._thread_started:
                self.runthread_over = True
            self.stop()
            raise

    def get_rtsp_url(self):
        try:
            return self.rtspserver.rtspserver_getrtspurl(RTSP_SESSION_NAME)
        except TypeError:
            return self.rtspserver.rtspserver_getrtspurl()

    def is_healthy(self):
        return self.start_stream and self.thread_error is None

    def _do_rtsp_stream(self):
        stream_data = StreamData()
        frame_count = 0
        frames_since_idr_request = 0
        stream_failure_count = 0
        send_failure_count = 0
        idr_failure_count = 0
        request_idr_supported = hasattr(self.encoder, "RequestIDR")

        if request_idr_supported:
            try:
                self.encoder.RequestIDR()
                print("[RTSP] Initial IDR requested")
            except Exception as error:
                idr_failure_count += 1
                self._print_repeated_warning(
                    "[RTSP] RequestIDR failed",
                    idr_failure_count,
                    error,
                )
        else:
            print("[RTSP] RequestIDR unavailable; continuing with encoder GOP")

        try:
            while self.start_stream:
                _exitpoint()
                stream_acquired = False
                stream_succeeded = False
                try:
                    try:
                        result = self.encoder.GetStream(
                            self.venc_chn,
                            stream_data,
                            VENC_STREAM_TIMEOUT_MS,
                        )
                    except Exception as error:
                        stream_failure_count += 1
                        self._print_repeated_warning(
                            "[RTSP] GetStream exception repeated",
                            stream_failure_count,
                            error,
                        )
                        time.sleep_ms(10)
                        continue

                    if result is not None and result != 0:
                        stream_failure_count += 1
                        self._print_repeated_warning(
                            "[RTSP] GetStream returned an error",
                            stream_failure_count,
                            result,
                        )
                        time.sleep_ms(10)
                        continue

                    stream_acquired = True
                    stream_failure_count = 0
                    for pack_index in range(stream_data.pack_cnt):
                        payload_size = stream_data.data_size[pack_index]
                        payload = bytes(
                            uctypes.bytearray_at(
                                stream_data.data[pack_index],
                                payload_size,
                            )
                        )
                        try:
                            send_result = self.rtspserver.rtspserver_sendvideodata(
                                RTSP_SESSION_NAME,
                                payload,
                                payload_size,
                                RTSP_SEND_TIMESTAMP,
                            )
                        except Exception as error:
                            send_failure_count += 1
                            self._print_repeated_warning(
                                "[RTSP] Send exception repeated",
                                send_failure_count,
                                error,
                            )
                            continue

                        if send_result is not None and send_result != 0:
                            send_failure_count += 1
                            self._print_repeated_warning(
                                "[RTSP] Send returned an error",
                                send_failure_count,
                                send_result,
                            )
                        else:
                            send_failure_count = 0

                    stream_succeeded = True

                except Exception as error:
                    send_failure_count += 1
                    self._print_repeated_warning(
                        "[RTSP] Frame processing exception repeated",
                        send_failure_count,
                        error,
                    )
                    gc.collect()
                finally:
                    if stream_acquired:
                        try:
                            self.encoder.ReleaseStream(
                                self.venc_chn,
                                stream_data,
                            )
                        except Exception as error:
                            if self.thread_error is None:
                                self.thread_error = error
                            self.start_stream = False
                            _print_exception("[RTSP] ReleaseStream error:", error)

                if not self.start_stream:
                    break
                if not stream_succeeded:
                    time.sleep_ms(10)
                    continue

                frame_count += 1
                frames_since_idr_request += 1
                if (
                    request_idr_supported
                    and frames_since_idr_request >= IDR_REQUEST_INTERVAL_FRAMES
                ):
                    try:
                        self.encoder.RequestIDR()
                        idr_failure_count = 0
                    except Exception as error:
                        idr_failure_count += 1
                        self._print_repeated_warning(
                            "[RTSP] RequestIDR failed",
                            idr_failure_count,
                            error,
                        )
                    frames_since_idr_request = 0

                if frame_count >= 60:
                    gc.collect()
                    frame_count = 0
        finally:
            self.start_stream = False
            self.runthread_over = True

    def stop(self):
        self.start_stream = False

        if self._thread_started and not self.runthread_over:
            started_ms = time.ticks_ms()
            while not self.runthread_over:
                if (
                    time.ticks_diff(time.ticks_ms(), started_ms)
                    >= STREAM_THREAD_STOP_TIMEOUT_MS
                ):
                    print("[RTSP] Stream thread stop timed out")
                    return False
                time.sleep_ms(50)

        self._thread_started = False

        if self._sensor_started:
            self._safe_cleanup("Sensor.stop", self.sensor.stop)
            self._sensor_started = False

        if self.link is not None:
            link = self.link
            self.link = None
            if hasattr(link, "destroy"):
                self._safe_cleanup("MediaManager.link.destroy", link.destroy)
            del link

        if self._encoder_started:
            self._safe_cleanup(
                "Encoder.Stop",
                lambda: self.encoder.Stop(self.venc_chn),
            )
            self._encoder_started = False

        if self._encoder_created:
            self._safe_cleanup(
                "Encoder.Destroy",
                lambda: self.encoder.Destroy(self.venc_chn),
            )
            self._encoder_created = False

        if self._media_initialized:
            self._safe_cleanup("MediaManager.deinit", MediaManager.deinit)
            self._media_initialized = False

        if self._rtsp_started:
            self._safe_cleanup(
                "rtspserver_stop",
                self.rtspserver.rtspserver_stop,
            )
            self._rtsp_started = False

        if self._session_created and hasattr(
            self.rtspserver,
            "rtspserver_destroysession",
        ):
            self._safe_cleanup(
                "rtspserver_destroysession",
                lambda: self.rtspserver.rtspserver_destroysession(
                    RTSP_SESSION_NAME
                ),
            )
            self._session_created = False

        if self._rtsp_initialized:
            self._safe_cleanup(
                "rtspserver_deinit",
                self.rtspserver.rtspserver_deinit,
            )
            self._rtsp_initialized = False

        if not self._rtsp_destroyed and hasattr(
            self.rtspserver,
            "rtspserver_destroy",
        ):
            self._safe_cleanup(
                "rtspserver_destroy",
                self.rtspserver.rtspserver_destroy,
            )
            self._rtsp_destroyed = True

        return True


def main():
    if hasattr(os, "EXITPOINT_ENABLE") and hasattr(os, "exitpoint"):
        os.exitpoint(os.EXITPOINT_ENABLE)

    while True:
        server = None
        stop_requested = False
        try:
            sta, config = connect_wifi()
            server = RtspServer()
            server.start()

            actual_url = server.get_rtsp_url()
            print("[RTSP] Started:", actual_url)
            print(
                "[RTSP] Player URL: rtsp://%s:%d/%s"
                % (config[0], RTSP_PORT, RTSP_SESSION_NAME)
            )

            while sta.isconnected() and server.is_healthy():
                _sleep_ms(500)

            if not sta.isconnected():
                print("[WIFI] Link lost")
            elif server.thread_error is not None:
                _print_exception("[RTSP] Restarting after error:", server.thread_error)
        except KeyboardInterrupt:
            print("[BOOT] Stopped by user")
            stop_requested = True
        except Exception as error:
            _print_exception("[BOOT] Runtime error:", error)
        finally:
            if server is not None and not server.stop():
                print("[BOOT] Unsafe to restart; reboot K230 to recover")
                stop_requested = True

        if stop_requested:
            break

        gc.collect()
        print("[BOOT] Retrying Wi-Fi and RTSP in 3 seconds")
        _sleep_ms(RECONNECT_DELAY_MS)


main()
