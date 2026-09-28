import unittest

from bluetooth_protocol import (
    CMD_HELLO,
    CMD_WRITE_PARAM,
    FrameParser,
    decode_status,
    encode_frame,
)


class ProtocolTest(unittest.TestCase):
    def test_round_trip_and_split_input(self):
        encoded = encode_frame(CMD_HELLO, 7)
        parser = FrameParser()
        self.assertEqual(parser.feed(encoded[:3]), [])
        frames = parser.feed(encoded[3:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].command, CMD_HELLO)
        self.assertEqual(frames[0].sequence, 7)

    def test_crc_error_then_resynchronize(self):
        bad = bytearray(encode_frame(CMD_WRITE_PARAM, 1, b"\x01\xFA\x00\x00\x00"))
        bad[-1] ^= 0x80
        good = encode_frame(CMD_HELLO, 2)
        parser = FrameParser()
        frames = parser.feed(bytes(bad) + good)
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual([frame.sequence for frame in frames], [2])

    def test_status_size_is_exact(self):
        status = decode_status(bytes(30))
        self.assertEqual(status["elapsed_ms"], 0)
        with self.assertRaises(ValueError):
            decode_status(bytes(29))


if __name__ == "__main__":
    unittest.main()
