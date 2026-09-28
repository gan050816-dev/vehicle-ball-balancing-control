"""Task-3 Bluetooth tuning protocol shared by the Windows upper computer."""

from dataclasses import dataclass
import struct

HEAD = b"\xB5\x62"
VERSION = 1
MAX_PAYLOAD = 32

CMD_HELLO = 0x01
CMD_READ_PARAM = 0x10
CMD_WRITE_PARAM = 0x11
CMD_READ_STATUS = 0x20
CMD_SET_STREAM = 0x21

RSP_INFO = 0x81
RSP_PARAM = 0x90
RSP_STATUS = 0xA0
RSP_ACK = 0xA1
RSP_ERROR = 0xE0

STATUS_OK = 0


@dataclass(frozen=True)
class Frame:
    command: int
    sequence: int
    payload: bytes


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(command: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes((VERSION, command & 0xFF, sequence & 0xFF, len(payload))) + payload
    return HEAD + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.format_errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            head_index = self.buffer.find(HEAD)
            if head_index < 0:
                if self.buffer[-1:] == HEAD[:1]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if head_index:
                del self.buffer[:head_index]
            if len(self.buffer) < 6:
                break
            version, command, sequence, payload_length = self.buffer[2:6]
            if version != VERSION or payload_length > MAX_PAYLOAD:
                self.format_errors += 1
                del self.buffer[0]
                continue
            frame_length = 8 + payload_length
            if len(self.buffer) < frame_length:
                break
            body = bytes(self.buffer[2 : 6 + payload_length])
            received_crc = struct.unpack_from("<H", self.buffer, 6 + payload_length)[0]
            if received_crc != crc16_ccitt(body):
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append(Frame(command, sequence, bytes(self.buffer[6 : 6 + payload_length])))
            del self.buffer[:frame_length]
        return frames


STATUS_STRUCT = struct.Struct("<BBBBHhhhhHHHHhhHH")


def decode_status(payload: bytes) -> dict[str, int]:
    if len(payload) != STATUS_STRUCT.size:
        raise ValueError(f"invalid status size: {len(payload)}")
    names = (
        "app_state", "phase", "stepper_status", "flags", "elapsed_ms",
        "raw_x", "filtered_x", "velocity", "command_angle",
        "commanded_rack", "measured_rack", "positive_time",
        "negative_time", "positive_velocity", "negative_velocity",
        "protocol_errors", "rx_overflows",
    )
    return dict(zip(names, STATUS_STRUCT.unpack(payload)))
