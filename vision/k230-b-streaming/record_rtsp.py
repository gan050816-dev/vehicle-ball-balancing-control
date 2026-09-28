"""Record a K230 RTSP H.264 stream without decoding or re-encoding."""

import argparse
import os
import shutil
import subprocess
import sys
import threading
import time


DEFAULT_RTSP_URL = "rtsp://192.168.1.200:8554/video"
RTSP_IO_TIMEOUT_US = 10_000_000
RTSP_IO_TIMEOUT_SECONDS = RTSP_IO_TIMEOUT_US / 1_000_000
RECONNECT_DELAY_SECONDS = 1.0


def find_ffmpeg():
    """Return the bundled imageio FFmpeg executable, or a PATH fallback."""
    try:
        import imageio_ffmpeg

        executable = imageio_ffmpeg.get_ffmpeg_exe()
        if executable and os.path.isfile(executable):
            return executable
    except Exception as exc:
        print(f"[WARN] imageio-ffmpeg is unavailable: {exc}")

    executable = shutil.which("ffmpeg")
    if executable:
        return executable
    raise RuntimeError("FFmpeg was not found (install imageio-ffmpeg or add ffmpeg to PATH)")


def build_ffmpeg_command(ffmpeg_exe, rtsp_url, output_file, duration=0):
    """Build a direct H.264 stream-copy command compatible with the BAT file."""
    command = [
        ffmpeg_exe,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-stats",
        "-n",
        # FFmpeg RTSP documentation: TCP interleaves RTP in the control channel.
        # https://ffmpeg.org/ffmpeg-protocols.html#rtsp
        "-rtsp_transport",
        "tcp",
        "-timeout",
        str(RTSP_IO_TIMEOUT_US),
        "-fflags",
        "+genpts",
        "-i",
        rtsp_url,
        "-map",
        "0:v:0",
        "-an",
        # FFmpeg streamcopy performs no decode, filter, or re-encode operation.
        # https://ffmpeg.org/ffmpeg.html#Streamcopy
        "-c:v",
        "copy",
        "-avoid_negative_ts",
        "make_zero",
    ]

    if duration and duration > 0:
        command.extend(["-t", str(duration)])

    if os.path.splitext(output_file)[1].lower() in (".mp4", ".mov"):
        # Move the MP4 index to the beginning after a clean close.
        # https://ffmpeg.org/ffmpeg-formats.html#mov_002c-mp4_002c-ismv
        command.extend(["-movflags", "+faststart"])

    command.append(output_file)
    return command


def get_parts_directory(output_file):
    """Return the dedicated recovery directory for one final recording."""
    output_root, _ = os.path.splitext(os.path.abspath(output_file))
    return output_root + "_parts"


def get_part_path(parts_directory, part_number):
    """Return a numbered MPEG-TS path that survives interrupted recording."""
    return os.path.join(parts_directory, f"part{part_number:04d}.ts")


def quote_concat_path(path):
    """Quote an absolute path for an FFmpeg concat-demuxer manifest."""
    normalized = os.path.abspath(path).replace("\\", "/")
    return "'" + normalized.replace("'", "'\\''") + "'"


def write_concat_manifest(parts_directory, part_paths):
    """Write a manifest that joins recorded parts in chronological order."""
    manifest_path = os.path.join(parts_directory, "concat.txt")
    with open(manifest_path, "w", encoding="utf-8", newline="\n") as manifest:
        for part_path in part_paths:
            manifest.write(f"file {quote_concat_path(part_path)}\n")
    return manifest_path


def build_merge_command(ffmpeg_exe, manifest_path, output_file):
    """Build a stream-copy command that remuxes all recovered parts to MP4."""
    command = [
        ffmpeg_exe,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-stats",
        "-nostdin",
        "-n",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        manifest_path,
        "-map",
        "0:v:0",
        "-an",
        "-c:v",
        "copy",
    ]
    if os.path.splitext(output_file)[1].lower() in (".mp4", ".mov"):
        command.extend(["-movflags", "+faststart"])
    command.append(output_file)
    return command


def finalize_parts(ffmpeg_exe, parts_directory, part_paths, output_file):
    """Remux recovered parts and delete them only after verified success."""
    try:
        manifest_path = write_concat_manifest(parts_directory, part_paths)
    except OSError as exc:
        print(f"[ERROR] Cannot write merge manifest: {exc}")
        print(f"[RECOVERY] Parts kept in: {parts_directory}")
        return 8
    merge_command = build_merge_command(ffmpeg_exe, manifest_path, output_file)
    print(f"[MERGE] Joining {len(part_paths)} recovered part(s) ...")

    try:
        merge_result = subprocess.run(merge_command, check=False)
    except OSError as exc:
        print(f"[ERROR] Cannot start FFmpeg merge: {exc}")
        print(f"[RECOVERY] Parts kept in: {parts_directory}")
        return 8

    if merge_result.returncode != 0:
        print(f"[ERROR] Merge failed with code {merge_result.returncode}")
        print(f"[RECOVERY] Parts kept in: {parts_directory}")
        return 8
    if not os.path.isfile(output_file) or os.path.getsize(output_file) == 0:
        print("[ERROR] Merge reported success but produced no video data")
        print(f"[RECOVERY] Parts kept in: {parts_directory}")
        return 8

    cleanup_paths = list(part_paths) + [manifest_path]
    cleanup_failed = False
    for cleanup_path in cleanup_paths:
        try:
            os.remove(cleanup_path)
        except OSError as exc:
            cleanup_failed = True
            print(f"[WARN] Could not remove temporary file {cleanup_path}: {exc}")
    if not cleanup_failed:
        try:
            os.rmdir(parts_directory)
        except OSError as exc:
            print(f"[WARN] Could not remove temporary folder: {exc}")
    return 0


def keep_nonempty_part(part_path, part_paths):
    """Remember a non-empty part, or discard an empty generated placeholder."""
    if not os.path.exists(part_path):
        return False
    if os.path.getsize(part_path) > 0:
        if part_path not in part_paths:
            part_paths.append(part_path)
        return True
    try:
        os.remove(part_path)
    except OSError as exc:
        print(f"[WARN] Could not remove empty part {part_path}: {exc}")
    return False


def request_graceful_stop(process):
    """Ask FFmpeg to finalize the container and stop."""
    if process.poll() is not None:
        return
    try:
        process.stdin.write(b"q\n")
        process.stdin.flush()
    except (BrokenPipeError, OSError, ValueError):
        pass


def watch_for_q(stop_event, stop_state, process_state):
    """Watch the Windows console for Q across every reconnect attempt."""
    try:
        import msvcrt
    except ImportError:
        return

    while not stop_event.is_set():
        if msvcrt.kbhit():
            key = msvcrt.getwch()
            if key.lower() == "q":
                stop_state["reason"] = "Q key pressed"
                stop_event.set()
                print("\n[STOP] Q pressed; finalizing the video file ...")
                process = process_state.get("process")
                if process is not None:
                    request_graceful_stop(process)
                return
        time.sleep(0.05)


def wait_for_attempt(process, stop_event, stop_state, deadline=None):
    """Wait for one FFmpeg attempt while honoring Q and total duration."""
    stop_requested = False
    while process.poll() is None:
        if deadline is not None and time.monotonic() >= deadline:
            stop_state["reason"] = "configured duration reached"
            stop_event.set()
        if stop_event.is_set() and not stop_requested:
            request_graceful_stop(process)
            stop_requested = True
        try:
            return process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            continue
    return process.returncode


def record(rtsp_url, output_file, duration=0):
    output_file = os.path.abspath(output_file)
    output_directory = os.path.dirname(output_file)
    parts_directory = get_parts_directory(output_file)

    if not os.path.isdir(output_directory):
        print(f"[ERROR] Output directory does not exist: {output_directory}")
        return 2
    if os.path.exists(output_file):
        print(f"[ERROR] Refusing to overwrite existing file: {output_file}")
        return 3
    if os.path.exists(parts_directory):
        print(f"[ERROR] Recovery folder already exists: {parts_directory}")
        print("[ERROR] Keep it for recovery or rename it before recording again")
        return 3

    try:
        ffmpeg_exe = find_ffmpeg()
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return 4

    try:
        os.mkdir(parts_directory)
    except OSError as exc:
        print(f"[ERROR] Cannot create recovery folder: {exc}")
        return 5

    print(f"[RECORD] Source: {rtsp_url}")
    print("[RECORD] Mode: direct H.264 stream copy (no BGR conversion)")
    print(f"[RECORD] Output: {output_file}")
    print(f"[RECOVERY] Live parts: {parts_directory}")
    if duration and duration > 0:
        print(f"[LIMIT] Duration: {duration:g} seconds")
        automatic_stop_conditions = "configured duration only"
    else:
        print("[LIMIT] Duration: unlimited")
        automatic_stop_conditions = "none"
    print(f"[LIMIT] RTSP I/O timeout: {RTSP_IO_TIMEOUT_SECONDS:g} seconds")
    print(
        f"[LIMIT] Auto reconnect: enabled, unlimited, "
        f"retry every {RECONNECT_DELAY_SECONDS:g} second(s)"
    )
    print("[LIMIT] Existing-file overwrite: disabled")
    print("[LIMIT] Manual stop: Q or Ctrl+C")
    print(f"[LIMIT] Automatic stop: {automatic_stop_conditions}")
    print("[RECORD] FFmpeg warnings/errors remain visible below")
    print()

    started_at = time.monotonic()
    deadline = started_at + duration if duration and duration > 0 else None
    stop_event = threading.Event()
    stop_state = {"reason": None}
    process_state = {"process": None}
    part_paths = []
    part_number = 0
    current_part_path = None
    key_thread = threading.Thread(
        target=watch_for_q,
        args=(stop_event, stop_state, process_state),
        daemon=True,
    )
    key_thread.start()

    try:
        while not stop_event.is_set():
            if deadline is not None and time.monotonic() >= deadline:
                stop_state["reason"] = "configured duration reached"
                stop_event.set()
                break

            part_number += 1
            current_part_path = get_part_path(parts_directory, part_number)
            command = build_ffmpeg_command(
                ffmpeg_exe,
                rtsp_url,
                current_part_path,
                0,
            )
            print(f"[CONNECT] Attempt {part_number}: {rtsp_url}")
            try:
                process = subprocess.Popen(
                    command,
                    stdin=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError as exc:
                print(f"[WARN] Cannot start FFmpeg: {exc}")
                print(f"[RECONNECT] Retrying in {RECONNECT_DELAY_SECONDS:g}s ...")
                if stop_event.wait(RECONNECT_DELAY_SECONDS):
                    break
                continue

            process_state["process"] = process
            try:
                try:
                    return_code = wait_for_attempt(
                        process,
                        stop_event,
                        stop_state,
                        deadline,
                    )
                except KeyboardInterrupt:
                    stop_state["reason"] = "Ctrl+C received"
                    stop_event.set()
                    print(
                        "\n[STOP] Ctrl+C received; "
                        "finalizing the current part ..."
                    )
                    request_graceful_stop(process)
                    try:
                        return_code = process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        print(
                            "[WARN] FFmpeg did not stop in 3 seconds; "
                            "terminating it"
                        )
                        process.terminate()
                        return_code = process.wait()
            finally:
                process_state["process"] = None
                if process.stdin is not None:
                    try:
                        process.stdin.close()
                    except OSError:
                        pass

            if keep_nonempty_part(current_part_path, part_paths):
                part_size_mb = os.path.getsize(current_part_path) / (1024 * 1024)
                print(f"[PART] Kept part {part_number}: {part_size_mb:.2f} MB")

            if stop_event.is_set():
                break
            if return_code == 0:
                print("[RECONNECT] RTSP source ended; reconnecting")
            else:
                print(f"[RECONNECT] FFmpeg exited with code {return_code}")
            print(f"[RECONNECT] Retrying in {RECONNECT_DELAY_SECONDS:g}s ...")
            if stop_event.wait(RECONNECT_DELAY_SECONDS):
                break
    except KeyboardInterrupt:
        stop_state["reason"] = "Ctrl+C received"
        stop_event.set()
        print("\n[STOP] Ctrl+C received; finalizing the video file ...")
        process = process_state.get("process")
        if process is not None:
            request_graceful_stop(process)
            try:
                return_code = process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print("[WARN] FFmpeg did not stop in 3 seconds; terminating it")
                process.terminate()
                return_code = process.wait()
            if current_part_path is not None:
                keep_nonempty_part(current_part_path, part_paths)
    finally:
        stop_event.set()
        active_process = process_state.get("process")
        if active_process is not None and active_process.poll() is None:
            request_graceful_stop(active_process)
            try:
                active_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                active_process.terminate()
        process_state["process"] = None

    elapsed = time.monotonic() - started_at
    stop_reason = stop_state["reason"] or "recording loop ended"
    print(f"[STOP_REASON] {stop_reason}")
    if not part_paths:
        print("[ERROR] No video data was received")
        try:
            os.rmdir(parts_directory)
        except OSError:
            print(f"[RECOVERY] Session folder kept: {parts_directory}")
        return 7

    finalize_result = finalize_parts(
        ffmpeg_exe,
        parts_directory,
        part_paths,
        output_file,
    )
    if finalize_result != 0:
        return finalize_result

    size_mb = os.path.getsize(output_file) / (1024 * 1024)
    print(f"[DONE] Saved {size_mb:.2f} MB in {elapsed:.1f}s")
    print(f"[DONE] File: {output_file}")
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    record_parser = subparsers.add_parser("record", help="record an RTSP stream")
    record_parser.add_argument("--url", default=DEFAULT_RTSP_URL)
    record_parser.add_argument("--output", default="video.mp4")
    record_parser.add_argument("--time", type=float, default=0)

    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.command == "record":
        return record(args.url, args.output, args.time)
    return 1


if __name__ == "__main__":
    sys.exit(main())
