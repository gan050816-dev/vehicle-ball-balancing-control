# -*- coding: utf-8 -*-
"""
实时钢珠检测并通过二进制协议向 MSPM0G3507 上报位置。

物理链路：
K230 GPIO9 / UART1_TXD -> MSPM0 PB3 / UART3_RX
K230 GPIO10 / UART1_RXD <- MSPM0 PB2 / UART3_TX
115200, 8N1, TTL, common ground
"""

import gc
import time

from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *
from machine import UART, FPIOA


# UART1 on K230, connected to UART3 on MSPM0G3507.
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD)
fpioa.set_function(10, fpioa.UART1_RXD)
uart = UART(
    1,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
    timeout=0,
)


# Protocol: A5 5A command length payload checksum.
FRAME_HEAD = b"\xA5\x5A"
CMD_BALL_STATE = 0x10
CMD_CAPTURE_RESULT = 0x11
CMD_CAPTURE_REQUEST = 0x90
CMD_USE_DEFAULT_ORIGIN = 0x91

BALL_FLAG_VALID = 0x01
BALL_FLAG_CALIBRATED = 0x02
BALL_FLAG_MULTI = 0x04
BALL_FLAG_TARGET_CENTER_VALID = 0x08
BALL_X_ABS_MAX_TENTHS_MM = 1500

CAPTURE_STATUS_OK = 0
CAPTURE_STATUS_TIMEOUT = 1
CAPTURE_STATUS_UNSTABLE = 2
CAPTURE_SAMPLE_COUNT = 15
CAPTURE_MAX_SPREAD_PX = 3
CAPTURE_TIMEOUT_MS = 2000

SEND_PERIOD_MS = 20
PRINT_UART_DATA = False

_tx_sequence = 0
_last_sent_sequence = 0
_last_sent_x_tenths_mm = 0
_last_sent_confidence_permille = 0
_last_sent_valid = False
_last_tx_attempted = False
_last_tx_success = False

_rx_state = 0
_rx_command = 0
_rx_length = 0
_rx_payload = bytearray()
_rx_checksum = 0

_target_center_x = 640
_target_center_y = 360
_target_center_valid = False
_default_origin_center_x = 0
_default_origin_center_y = 0
_default_origin_center_valid = False
_fixed_positive_target_x = 0
_fixed_negative_target_x = 0
_fixed_target_lines_valid = False
_fixed_calibration_valid_frames = 0
_capture_active = False
_capture_request_id = 0
_capture_started_ms = 0
_capture_samples = []
_capture_unstable_windows = 0
_capture_status_text = "IDLE"


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def pack_u16_le(value):
    value &= 0xFFFF
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def send_frame(command, payload=b""):
    checksum = command + len(payload)
    for value in payload:
        checksum += value
    checksum &= 0xFF

    frame = (
        FRAME_HEAD
        + bytes((command, len(payload)))
        + bytes(payload)
        + bytes((checksum,))
    )
    written = uart.write(frame)

    if PRINT_UART_DATA:
        print(
            "TX%s:" % ("" if written == len(frame) else " FAIL"),
            " ".join("%02X" % value for value in frame),
        )
    return written == len(frame)


def send_capture_result(request_id, status, center_x_px=0):
    payload = (
        bytes((request_id, status))
        + pack_u16_le(center_x_px)
    )
    return send_frame(CMD_CAPTURE_RESULT, payload)


def begin_capture(request_id):
    global _capture_active
    global _capture_request_id
    global _capture_started_ms
    global _capture_samples
    global _capture_unstable_windows
    global _capture_status_text

    _capture_active = True
    _capture_request_id = request_id
    _capture_started_ms = time.ticks_ms()
    _capture_samples = []
    _capture_unstable_windows = 0
    _capture_status_text = "WAIT"


def handle_received_frame(command, payload):
    global _capture_active
    global _capture_samples
    global _target_center_valid
    global _capture_status_text

    if command == CMD_CAPTURE_REQUEST and len(payload) == 1:
        begin_capture(payload[0])
    elif command == CMD_USE_DEFAULT_ORIGIN and len(payload) == 0:
        _capture_active = False
        _capture_samples = []
        _target_center_valid = False
        _capture_status_text = "DEFAULT"


def push_uart_rx_byte(value):
    global _rx_state
    global _rx_command
    global _rx_length
    global _rx_payload
    global _rx_checksum

    if _rx_state == 0:
        if value == 0xA5:
            _rx_state = 1
    elif _rx_state == 1:
        if value == 0x5A:
            _rx_state = 2
        elif value != 0xA5:
            _rx_state = 0
    elif _rx_state == 2:
        _rx_command = value
        _rx_checksum = value
        _rx_state = 3
    elif _rx_state == 3:
        _rx_length = value
        _rx_checksum = (_rx_checksum + value) & 0xFF
        _rx_payload = bytearray()
        if _rx_length == 0:
            _rx_state = 5
        elif _rx_length <= 8:
            _rx_state = 4
        else:
            _rx_state = 0
    elif _rx_state == 4:
        _rx_payload.append(value)
        _rx_checksum = (_rx_checksum + value) & 0xFF
        if len(_rx_payload) >= _rx_length:
            _rx_state = 5
    else:
        if value == _rx_checksum:
            handle_received_frame(_rx_command, _rx_payload)
        _rx_state = 1 if value == 0xA5 else 0


def poll_uart_commands():
    received = uart.read()
    if received is None:
        return
    for value in received:
        push_uart_rx_byte(value)


def update_capture(
    valid, center_x, center_y, confidence, candidate_count
):
    global _capture_active
    global _capture_samples
    global _capture_unstable_windows
    global _capture_status_text
    global _target_center_x
    global _target_center_y
    global _target_center_valid

    if not _capture_active:
        return

    if (
        valid
        and candidate_count == 1
        and confidence >= MIN_BALL_CONFIDENCE
    ):
        _capture_samples.append((center_x, center_y))
        if len(_capture_samples) >= CAPTURE_SAMPLE_COUNT:
            x_samples = [
                sample[0] for sample in _capture_samples
            ]
            y_samples = [
                sample[1] for sample in _capture_samples
            ]
            if (
                max(x_samples) - min(x_samples)
                <= CAPTURE_MAX_SPREAD_PX
                and max(y_samples) - min(y_samples)
                <= CAPTURE_MAX_SPREAD_PX
            ):
                x_samples.sort()
                y_samples.sort()
                _target_center_x = x_samples[
                    CAPTURE_SAMPLE_COUNT // 2
                ]
                _target_center_y = y_samples[
                    CAPTURE_SAMPLE_COUNT // 2
                ]
                _target_center_valid = True
                _capture_active = False
                result_sent = send_capture_result(
                    _capture_request_id,
                    CAPTURE_STATUS_OK,
                    _target_center_x,
                )
                _capture_status_text = (
                    "OK" if result_sent else "TXERR"
                )
                return
            _capture_samples = []
            _capture_unstable_windows += 1
    else:
        _capture_samples = []

    if (
        time.ticks_diff(time.ticks_ms(), _capture_started_ms)
        >= CAPTURE_TIMEOUT_MS
    ):
        status = CAPTURE_STATUS_TIMEOUT
        _capture_status_text = "TIMEOUT"
        if _capture_unstable_windows > 0:
            status = CAPTURE_STATUS_UNSTABLE
            _capture_status_text = "UNSTABLE"
        _capture_active = False
        _capture_samples = []
        if not send_capture_result(_capture_request_id, status, 0):
            _capture_status_text += "/TXERR"


def send_ball_state(
    valid,
    offset_cm=0.0,
    confidence=0.0,
    candidate_count=0,
    calibrated=False,
    target_center_valid=False,
):
    global _tx_sequence
    global _last_sent_sequence
    global _last_sent_x_tenths_mm
    global _last_sent_confidence_permille
    global _last_sent_valid
    global _last_tx_attempted
    global _last_tx_success

    if valid:
        scaled = offset_cm * 100.0
        if scaled >= 0:
            x_tenths_mm = int(scaled + 0.5)
        else:
            x_tenths_mm = int(scaled - 0.5)
        if abs(x_tenths_mm) > BALL_X_ABS_MAX_TENTHS_MM:
            valid = False

    if valid:
        confidence_permille = clamp(
            int(confidence * 1000.0 + 0.5), 0, 1000
        )
    else:
        x_tenths_mm = 0
        confidence_permille = 0

    flags = 0
    if valid:
        flags |= BALL_FLAG_VALID
    if calibrated:
        flags |= BALL_FLAG_CALIBRATED
    if candidate_count > 1:
        flags |= BALL_FLAG_MULTI
    if target_center_valid:
        flags |= BALL_FLAG_TARGET_CENTER_VALID

    timestamp_ms16 = time.ticks_ms() & 0xFFFF
    payload = (
        bytes((flags, _tx_sequence))
        + pack_u16_le(x_tenths_mm)
        + pack_u16_le(confidence_permille)
        + pack_u16_le(timestamp_ms16)
    )

    sent_sequence = _tx_sequence
    success = send_frame(CMD_BALL_STATE, payload)
    _last_sent_sequence = sent_sequence
    _last_sent_x_tenths_mm = x_tenths_mm
    _last_sent_confidence_permille = confidence_permille
    _last_sent_valid = valid
    _last_tx_attempted = True
    _last_tx_success = success
    _tx_sequence = (_tx_sequence + 1) & 0xFF
    return success


display_mode = "lt9611"
rgb888p_size = [1280, 720]

# Only accept a ball whose center lies in the middle third vertically.
ROI_Y1 = rgb888p_size[1] // 3
ROI_Y2 = rgb888p_size[1] * 2 // 3

# Infer centimeters per pixel from the known 1 cm ball. The frame-level
# CALIBRATED flag is asserted only while this runtime scale is available.
CENTER_X = rgb888p_size[0] // 2
REAL_DIAMETER_CM = 1.0
# The previous fixed marker was measured at physical +4 mm. Moving the
# zero marker 4 mm toward -X changes this offset from 6 mm to 2 mm.
DEFAULT_ORIGIN_OFFSET_CM = 0.2
# The fixed mechanical origin and the existing +5 cm line are confirmed
# accurate. Lock that positive span after a short settling window. Negative
# fixed-origin coordinates first retain the current calibration, then convert
# the current K230 output back to the measured ruler position.
FIXED_TARGET_ACTUAL_CM = 5.0
POSITIVE_TARGET_RAW_CM = 5.15
FIXED_CALIBRATION_SAMPLE_COUNT = 15
# Each pair is (fixed-base input, current K230 output), in cm.
FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM = (
    (0.0, 0.0),
    (-0.68, -1.0),
    (-2.13, -2.0),
    (-3.44, -3.0),
    (-4.38, -4.0),
    (-6.22, -5.0),
    (-9.02, -6.0),
)
# The negative-side measurement, in millimeters, is:
#     current_k230_mm = 0.6898 * actual_ruler_mm - 2.8626
# The fit starts at -10 mm. Join O to that first point continuously; at and
# beyond -10 mm use the measured inverse relation exactly. Zero and all
# positive coordinates bypass both negative corrections unchanged.
NEGATIVE_ACTUAL_TO_K230_SLOPE = 0.6898
NEGATIVE_ACTUAL_TO_K230_INTERCEPT_MM = -2.8626
NEGATIVE_FORMULA_START_ACTUAL_MM = -10.0
NEGATIVE_FORMULA_START_K230_MM = (
    NEGATIVE_ACTUAL_TO_K230_SLOPE
    * NEGATIVE_FORMULA_START_ACTUAL_MM
    + NEGATIVE_ACTUAL_TO_K230_INTERCEPT_MM
)
NEGATIVE_TARGET_ACTUAL_MM = -50.0
NEGATIVE_TARGET_CURRENT_OUTPUT_CM = (
    NEGATIVE_ACTUAL_TO_K230_SLOPE * NEGATIVE_TARGET_ACTUAL_MM
    + NEGATIVE_ACTUAL_TO_K230_INTERCEPT_MM
) / 10.0
TARGET_LINE_WIDTH_PX = 4
root_path = "/sdcard/mp_deployment_source/"
deploy_conf = read_json(root_path + "/deploy_config.json")
kmodel_path = root_path + deploy_conf["kmodel_path"]
labels = deploy_conf["categories"]
confidence_threshold = deploy_conf["confidence_threshold"]
nms_threshold = deploy_conf["nms_threshold"]
model_input_size = deploy_conf["img_size"]
nms_option = deploy_conf["nms_option"]
model_type = deploy_conf["model_type"]

anchors = []
if model_type == "AnchorBaseDet":
    anchors = (
        deploy_conf["anchors"][0]
        + deploy_conf["anchors"][1]
        + deploy_conf["anchors"][2]
    )

inference_mode = "video"
debug_mode = 0

GC_INTERVAL = 30
MIN_BALL_CONFIDENCE = 0.65
BOX_JITTER_DEADBAND_PX = 2
BOX_FOLLOW_PERCENT = 70


def fixed_offset_cm(center_x, origin_x, positive_target_x):
    positive_span_px = origin_x - positive_target_x
    if positive_span_px <= 0:
        return 0.0
    return (
        (origin_x - center_x)
        * FIXED_TARGET_ACTUAL_CM
        / positive_span_px
    )


def current_fixed_negative_output_cm(offset_cm):
    if offset_cm >= 0.0:
        return offset_cm

    previous_input_cm, previous_output_cm = (
        FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[0]
    )
    for input_cm, output_cm in FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[1:]:
        if offset_cm >= input_cm:
            return previous_output_cm + (
                (offset_cm - previous_input_cm)
                * (output_cm - previous_output_cm)
                / (input_cm - previous_input_cm)
            )
        previous_input_cm = input_cm
        previous_output_cm = output_cm

    input_cm, output_cm = FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[-1]
    previous_input_cm, previous_output_cm = (
        FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[-2]
    )
    return output_cm + (
        (offset_cm - input_cm)
        * (output_cm - previous_output_cm)
        / (input_cm - previous_input_cm)
    )


def fixed_input_cm_for_current_negative_output(current_output_cm):
    if current_output_cm >= 0.0:
        return current_output_cm

    previous_input_cm, previous_output_cm = (
        FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[0]
    )
    for input_cm, output_cm in FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[1:]:
        if current_output_cm >= output_cm:
            return previous_input_cm + (
                (current_output_cm - previous_output_cm)
                * (input_cm - previous_input_cm)
                / (output_cm - previous_output_cm)
            )
        previous_input_cm = input_cm
        previous_output_cm = output_cm

    input_cm, output_cm = FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[-1]
    previous_input_cm, previous_output_cm = (
        FIXED_BASE_TO_CURRENT_NEGATIVE_POINTS_CM[-2]
    )
    return input_cm + (
        (current_output_cm - output_cm)
        * (input_cm - previous_input_cm)
        / (output_cm - previous_output_cm)
    )


def correct_current_negative_output_cm(current_output_cm):
    if current_output_cm >= 0.0:
        return current_output_cm

    current_output_mm = current_output_cm * 10.0
    if current_output_mm >= NEGATIVE_FORMULA_START_K230_MM:
        return (
            current_output_mm
            * NEGATIVE_FORMULA_START_ACTUAL_MM
            / NEGATIVE_FORMULA_START_K230_MM
            / 10.0
        )
    return (
        (current_output_mm - NEGATIVE_ACTUAL_TO_K230_INTERCEPT_MM)
        / NEGATIVE_ACTUAL_TO_K230_SLOPE
        / 10.0
    )


def correct_fixed_negative_offset_cm(offset_cm):
    return correct_current_negative_output_cm(
        current_fixed_negative_output_cm(offset_cm)
    )


def main():
    global _default_origin_center_x
    global _default_origin_center_y
    global _default_origin_center_valid
    global _fixed_positive_target_x
    global _fixed_negative_target_x
    global _fixed_target_lines_valid
    global _fixed_calibration_valid_frames

    pl = None
    det_app = None
    filtered_box = None
    average_box_width = 0
    fps_frame_count = 0
    gc_frame_count = 0
    fps_started_ms = time.ticks_ms()
    last_send_ms = time.ticks_ms()

    try:
        pl = PipeLine(
            rgb888p_size=rgb888p_size,
            display_mode=display_mode,
        )
        pl.create()
        display_size = pl.get_display_size()

        det_app = DetectionApp(
            inference_mode,
            kmodel_path,
            labels,
            model_input_size,
            anchors,
            model_type,
            confidence_threshold,
            nms_threshold,
            rgb888p_size,
            display_size,
            debug_mode=debug_mode,
        )
        det_app.config_preprocess()

        while True:
            poll_uart_commands()
            img = pl.get_frame()
            result = det_app.run(img)
            pl.osd_img.clear()

            boxes_nd = result.get("boxes")
            scores_nd = result.get("scores")
            valid_boxes = []
            if boxes_nd is not None:
                for index, box in enumerate(boxes_nd):
                    box_center_y = (int(box[1]) + int(box[3])) // 2
                    if ROI_Y1 <= box_center_y < ROI_Y2:
                        score = 0.0
                        if scores_nd is not None:
                            score = float(scores_nd[index])
                        if score >= MIN_BALL_CONFIDENCE:
                            valid_boxes.append((box, score))

            transmit_valid = False
            transmit_offset_cm = 0.0
            transmit_confidence = 0.0
            transmit_center_x = 0
            transmit_center_y = 0
            cm_per_pixel = 0.0

            if valid_boxes:
                box, confidence = max(
                    valid_boxes, key=lambda item: item[1]
                )
                x1 = int(box[0])
                y1 = int(box[1])
                x2 = int(box[2])
                y2 = int(box[3])

                if filtered_box is None:
                    filtered_box = (x1, y1, x2, y2)
                else:
                    old_x1, old_y1, old_x2, old_y2 = filtered_box
                    raw_center_x = (x1 + x2) // 2
                    raw_center_y = (y1 + y2) // 2
                    old_center_x = (old_x1 + old_x2) // 2
                    old_center_y = (old_y1 + old_y2) // 2

                    if (
                        abs(raw_center_x - old_center_x)
                        > BOX_JITTER_DEADBAND_PX
                        or abs(raw_center_y - old_center_y)
                        > BOX_JITTER_DEADBAND_PX
                    ):
                        keep_percent = 100 - BOX_FOLLOW_PERCENT
                        filtered_box = (
                            (
                                old_x1 * keep_percent
                                + x1 * BOX_FOLLOW_PERCENT
                            )
                            // 100,
                            (
                                old_y1 * keep_percent
                                + y1 * BOX_FOLLOW_PERCENT
                            )
                            // 100,
                            (
                                old_x2 * keep_percent
                                + x2 * BOX_FOLLOW_PERCENT
                            )
                            // 100,
                            (
                                old_y2 * keep_percent
                                + y2 * BOX_FOLLOW_PERCENT
                            )
                            // 100,
                        )

                x1, y1, x2, y2 = filtered_box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                box_width = x2 - x1

                if average_box_width == 0:
                    average_box_width = box_width
                else:
                    average_box_width = (
                        average_box_width * 85 + box_width * 15
                    ) // 100

                cm_per_pixel = REAL_DIAMETER_CM / max(
                    average_box_width, 1
                )
                transmit_center_x = center_x
                transmit_center_y = center_y
                transmit_confidence = confidence
                transmit_valid = True
            else:
                filtered_box = None

            if transmit_valid and not _default_origin_center_valid:
                _default_origin_center_x = clamp(
                    int(
                        CENTER_X
                        - DEFAULT_ORIGIN_OFFSET_CM / cm_per_pixel
                    ),
                    0,
                    rgb888p_size[0] - 1,
                )
                _default_origin_center_y = transmit_center_y
                _default_origin_center_valid = True

            single_ball_for_fixed_calibration = (
                transmit_valid and len(valid_boxes) == 1
            )
            if (
                single_ball_for_fixed_calibration
                and _default_origin_center_valid
                and not _fixed_target_lines_valid
            ):
                _fixed_calibration_valid_frames += 1
                if (
                    _fixed_calibration_valid_frames
                    >= FIXED_CALIBRATION_SAMPLE_COUNT
                ):
                    positive_target_x = int(
                        _default_origin_center_x
                        - POSITIVE_TARGET_RAW_CM / cm_per_pixel
                    )
                    positive_span_px = (
                        _default_origin_center_x - positive_target_x
                    )
                    negative_target_input_cm = (
                        fixed_input_cm_for_current_negative_output(
                            NEGATIVE_TARGET_CURRENT_OUTPUT_CM
                        )
                    )
                    negative_target_x = int(
                        _default_origin_center_x
                        - negative_target_input_cm
                        * positive_span_px
                        / FIXED_TARGET_ACTUAL_CM
                    )
                    if (
                        positive_span_px > 0
                        and 0 <= positive_target_x < rgb888p_size[0]
                        and 0 <= negative_target_x < rgb888p_size[0]
                    ):
                        _fixed_positive_target_x = positive_target_x
                        _fixed_negative_target_x = negative_target_x
                        _fixed_target_lines_valid = True
            elif (
                not single_ball_for_fixed_calibration
                and not _fixed_target_lines_valid
            ):
                _fixed_calibration_valid_frames = 0

            update_capture(
                transmit_valid,
                transmit_center_x,
                transmit_center_y,
                transmit_confidence,
                len(valid_boxes),
            )
            if transmit_valid:
                if _target_center_valid:
                    # T6 capture defines its own zero and must not reuse the
                    # fixed mechanical-origin correction.
                    transmit_offset_cm = (
                        _target_center_x - transmit_center_x
                    ) * cm_per_pixel
                elif _fixed_target_lines_valid:
                    # Fixed mode uses the same locked pixel anchors as the
                    # OSD. Image left is +X; image right is -X.
                    transmit_offset_cm = (
                        correct_fixed_negative_offset_cm(
                            fixed_offset_cm(
                                transmit_center_x,
                                _default_origin_center_x,
                                _fixed_positive_target_x,
                            )
                        )
                    )

            effective_target_center_valid = (
                _target_center_valid or _default_origin_center_valid
            )
            coordinate_valid = transmit_valid and (
                _target_center_valid or _fixed_target_lines_valid
            )
            runtime_calibrated = coordinate_valid and cm_per_pixel > 0.0

            now_ms = time.ticks_ms()
            if (
                time.ticks_diff(now_ms, last_send_ms)
                >= SEND_PERIOD_MS
            ):
                last_send_ms = now_ms
                send_ball_state(
                    valid=coordinate_valid,
                    offset_cm=transmit_offset_cm,
                    confidence=transmit_confidence,
                    candidate_count=len(valid_boxes),
                    calibrated=runtime_calibrated,
                    target_center_valid=effective_target_center_valid,
                )

            display_width, display_height = display_size
            roi_display_y1 = (
                ROI_Y1 * display_height // rgb888p_size[1]
            )
            roi_display_y2 = (
                ROI_Y2 * display_height // rgb888p_size[1]
            )
            pl.osd_img.draw_rectangle(
                0,
                0,
                display_width,
                roi_display_y1,
                color=(255, 0, 0, 0),
                fill=True,
            )
            pl.osd_img.draw_rectangle(
                0,
                roi_display_y2,
                display_width,
                display_height - roi_display_y2,
                color=(255, 0, 0, 0),
                fill=True,
            )
            image_width, image_height = rgb888p_size
            if _target_center_valid:
                origin_center_x = _target_center_x
                origin_center_y = _target_center_y
                origin_center_valid = True
            else:
                origin_center_x = _default_origin_center_x
                origin_center_y = _default_origin_center_y
                origin_center_valid = _default_origin_center_valid

            # Fixed-origin lines and UART coordinates share these locked
            # pixel anchors. Ball-box scale changes cannot move the lines.
            if (
                not _target_center_valid
                and _fixed_target_lines_valid
            ):
                for (
                    target_image_x,
                    target_label,
                    target_color,
                ) in (
                    (
                        _fixed_positive_target_x,
                        "ACT:+5cm",
                        (255, 165, 0),
                    ),
                    (
                        _fixed_negative_target_x,
                        "ACT:-5cm",
                        (0, 255, 0),
                    ),
                ):
                    target_display_x = (
                        target_image_x * display_width // image_width
                    )
                    line_x = clamp(
                        target_display_x - TARGET_LINE_WIDTH_PX // 2,
                        0,
                        display_width - TARGET_LINE_WIDTH_PX,
                    )
                    pl.osd_img.draw_rectangle(
                        line_x,
                        roi_display_y1,
                        TARGET_LINE_WIDTH_PX,
                        roi_display_y2 - roi_display_y1,
                        color=target_color,
                        fill=True,
                    )
                    label_x = clamp(
                        target_display_x + 8,
                        0,
                        max(display_width - 260, 0),
                    )
                    pl.osd_img.draw_string_advanced(
                        label_x,
                        roi_display_y1 + 8,
                        18,
                        target_label,
                        color=target_color,
                    )

            if origin_center_valid:
                origin_display_x = (
                    origin_center_x * display_width // image_width
                )
                origin_display_y = (
                    origin_center_y * display_height // image_height
                )
                pl.osd_img.draw_circle(
                    origin_display_x,
                    origin_display_y,
                    8,
                    color=(255, 255, 0),
                    fill=True,
                )
            if transmit_valid:
                ball_display_x = (
                    transmit_center_x * display_width // image_width
                )
                ball_display_y = (
                    transmit_center_y * display_height // image_height
                )
                pl.osd_img.draw_circle(
                    ball_display_x,
                    ball_display_y,
                    5,
                    color=(255, 0, 0),
                    fill=True,
                )

            if _last_sent_valid:
                distance_text = "X:%+.1f mm" % (
                    _last_sent_x_tenths_mm / 10.0
                )
                state_text = "STATE:VALID CONF:%04d" % (
                    _last_sent_confidence_permille
                )
            else:
                distance_text = "X: NO BALL"
                state_text = "STATE:INVALID CONF:0000"

            tx_text = "SEQ:%03d UART_TX:%s" % (
                _last_sent_sequence,
                (
                    "WAIT"
                    if not _last_tx_attempted
                    else ("OK" if _last_tx_success else "ERR")
                ),
            )
            if _capture_active:
                capture_text = "CAP:WAIT %02d/%02d" % (
                    len(_capture_samples),
                    CAPTURE_SAMPLE_COUNT,
                )
            else:
                capture_text = "CAP:" + _capture_status_text
            if origin_center_valid:
                zero_text = "ZERO:%04d,%03dpx REQ:%03d" % (
                    origin_center_x,
                    origin_center_y,
                    _capture_request_id,
                )
            else:
                zero_text = "ZERO:----,---px REQ:%03d" % (
                    _capture_request_id
                )

            # The transmitted distance is dominant; diagnostic text is smaller.
            pl.osd_img.draw_string_advanced(
                16, 12, 52, distance_text, color=(0, 255, 0)
            )
            pl.osd_img.draw_string_advanced(
                16, 76, 22, state_text, color=(255, 255, 255)
            )
            pl.osd_img.draw_string_advanced(
                16, 104, 22, tx_text, color=(255, 255, 255)
            )
            pl.osd_img.draw_string_advanced(
                16, 132, 22, capture_text, color=(255, 255, 0)
            )
            pl.osd_img.draw_string_advanced(
                16, 160, 22, zero_text, color=(255, 255, 255)
            )

            fps_frame_count += 1
            fps_elapsed_ms = time.ticks_diff(
                time.ticks_ms(), fps_started_ms
            )
            if fps_elapsed_ms >= 1000:
                fps = fps_frame_count * 1000 // fps_elapsed_ms
                fps_frame_count = 0
                fps_started_ms = time.ticks_ms()
                print("[FPS] %d" % fps)

            gc_frame_count += 1
            if gc_frame_count >= GC_INTERVAL:
                gc_frame_count = 0
                gc.collect()

            pl.show_image()
    finally:
        if det_app is not None:
            det_app.deinit()
        if pl is not None:
            pl.destroy()
        gc.collect()


main()
