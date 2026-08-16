import struct
import unittest

from protocol import (
    MSG_CONFIG,
    MSG_CONTROL,
    MSG_TELEMETRY,
    STATE_NAMES,
    FrameParser,
    ServoTelemetry,
    crc16_ccitt,
    decode_telemetry,
    encode_control,
    encode_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_known_crc_vector(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_fragmented_and_back_to_back_frames(self) -> None:
        first = encode_control(7, 1000, -1000, 123, -456, 3)
        second = encode_frame(2, 8, struct.pack("<i", 123456))
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise" + first[:5]), [])
        frames = parser.feed(first[5:] + second)
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].message_type, MSG_CONTROL)
        self.assertEqual(frames[0].sequence, 7)
        self.assertEqual(struct.unpack("<hhhhH", frames[0].payload), (1000, -1000, 123, -456, 3))
        self.assertEqual(frames[1].message_type, MSG_CONFIG)
        self.assertEqual(struct.unpack("<i", frames[1].payload), (123456,))

    def test_lift_zero_state_names(self) -> None:
        self.assertEqual(STATE_NAMES[0], "未使能/零点无效")
        self.assertEqual(STATE_NAMES[2], "未使能/零点有效")
        self.assertIn("保留", STATE_NAMES[1])
        self.assertIn("保留", STATE_NAMES[5])

    def test_temperature_status_bit_is_labeled_diagnostic_only(self) -> None:
        diagnostic = ServoTelemetry(True, 0, 0x04, 40, 70, 12.0, 0, 0, 0)
        measured_hot = ServoTelemetry(True, 0, 0, 70, 70, 12.0, 0, 0, 0)

        self.assertIn("仅诊断", diagnostic.fault_text)
        self.assertNotIn("实测过温", diagnostic.fault_text)
        self.assertIn("实测过温", measured_hot.fault_text)

    def test_bad_crc_is_rejected_and_parser_recovers(self) -> None:
        damaged = bytearray(encode_control(1, 1, 2, 3, 4))
        damaged[-1] ^= 0x80
        valid = encode_control(2, 5, 6, 7, 8)
        frames = FrameParser().feed(damaged + valid)
        self.assertEqual([frame.sequence for frame in frames], [2])

    def test_telemetry_layout(self) -> None:
        payload = bytearray(85)
        payload[0] = 3
        struct.pack_into("<IHH", payload, 1, 1 << 2, 44, 12)
        payload[9:13] = bytes((0x0F, 0x02, 1, 1))
        struct.pack_into("<iiihhh", payload, 13, 4097, 5000, 12000, 10, -20, 30)
        for index in range(4):
            protocol_error = 0x20 if index == 0 else 0
            status_flags = 0x20 if index == 0 else 0
            struct.pack_into(
                "<BBBBBBhhh",
                payload,
                31 + index * 12,
                1,
                protocol_error,
                status_flags,
                40 + index,
                70,
                120,
                index,
                -index,
                100 + index,
            )
        struct.pack_into("<BBBBBB", payload, 79, 1, 1, 0x20, 0x20, 42, 70)
        frame = encode_frame(MSG_TELEMETRY, 1, payload)
        parsed = FrameParser().feed(frame)[0]
        telemetry = decode_telemetry(parsed.payload)
        self.assertEqual(telemetry.state, 3)
        self.assertEqual(telemetry.lift_position, 4097)
        self.assertEqual(telemetry.upper_limit, 12000)
        self.assertEqual(telemetry.wheel_speed, (10, -20, 30))
        self.assertTrue(all(servo.online for servo in telemetry.servos))
        self.assertEqual(telemetry.servos[0].status_flags, 0x20)
        self.assertEqual(telemetry.servos[0].temperature_limit_c, 70)
        self.assertTrue(telemetry.fault_snapshot.valid)
        self.assertIn("ID1", telemetry.fault_text)
        self.assertIn("过载", telemetry.fault_text)


if __name__ == "__main__":
    unittest.main()
