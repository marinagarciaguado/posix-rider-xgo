
import math
import unittest

from bridge.ipc_protocol import (
    PROTOCOL_VERSION,
    CMD_MSG_SIZE,
    TELEM_MSG_SIZE,
    MQ_PRIORITY_NORMAL,
    MQ_PRIORITY_EMERGENCY,
    XgoCommandType,
    XgoStatusFlags,
    XgoCmdMsg,
    XgoTelemMsg,
    create_cmd_message,
    create_telem_message,
    validate_cmd_message,
    validate_telem_message,
    encode_cmd_message,
    encode_telem_message,
    decode_cmd_message,
    decode_telem_message,
    cmd_mq_priority,
)


# ============================================================================
# Protocol layout tests
# ============================================================================

class TestProtocolLayout(unittest.TestCase):
    """Verify that Python structures match the expected C ABI."""

    def test_command_message_size(self):
        self.assertEqual(
            __import__("ctypes").sizeof(XgoCmdMsg),
            CMD_MSG_SIZE,
        )

    def test_telemetry_message_size(self):
        self.assertEqual(
            __import__("ctypes").sizeof(XgoTelemMsg),
            TELEM_MSG_SIZE,
        )

    def test_command_field_offsets(self):
        expected_offsets = {
            "sequence": 0,
            "version": 4,
            "command_type": 6,
            "vx": 8,
            "wz": 12,
            "height_z": 16,
            "action_id": 20,
            "timestamp_ns": 24,
        }

        for field, expected_offset in expected_offsets.items():
            with self.subTest(field=field):
                self.assertEqual(
                    getattr(XgoCmdMsg, field).offset,
                    expected_offset,
                )

    def test_telemetry_field_offsets(self):
        expected_offsets = {
            "sequence": 0,
            "version": 4,
            "status_flags": 6,
            "pitch_deg": 8,
            "roll_deg": 12,
            "yaw_deg": 16,
            "battery_pct": 20,
            "timestamp_ns": 24,
        }

        for field, expected_offset in expected_offsets.items():
            with self.subTest(field=field):
                self.assertEqual(
                    getattr(XgoTelemMsg, field).offset,
                    expected_offset,
                )


# ============================================================================
# Command message tests
# ============================================================================

class TestCommandMessage(unittest.TestCase):
    """Test command creation and validation."""

    def test_command_creation(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
            vx=0.5,
            wz=0.2,
        )

        self.assertEqual(msg.version, PROTOCOL_VERSION)
        self.assertEqual(msg.sequence, 1)
        self.assertEqual(msg.command_type, XgoCommandType.MOVE)

        self.assertTrue(validate_cmd_message(msg))

    def test_action_requires_id(self):
        with self.assertRaises(ValueError):
            create_cmd_message(
                XgoCommandType.ACTION,
                sequence=1,
            )

    def test_action_with_valid_id(self):
        msg = create_cmd_message(
            XgoCommandType.ACTION,
            sequence=1,
            action_id=5,
        )

        self.assertTrue(validate_cmd_message(msg))

    def test_invalid_command_version(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.version = 99

        self.assertFalse(validate_cmd_message(msg))

    def test_unknown_command_type_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.command_type = 999

        self.assertFalse(validate_cmd_message(msg))

    def test_nan_command_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.vx = math.nan

        self.assertFalse(validate_cmd_message(msg))

    def test_infinite_command_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.wz = math.inf

        self.assertFalse(validate_cmd_message(msg))

    def test_negative_infinity_command_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.height_z = -math.inf

        self.assertFalse(validate_cmd_message(msg))

    def test_zero_timestamp_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.timestamp_ns = 0

        self.assertFalse(validate_cmd_message(msg))

    def test_invalid_sequence_rejected(self):
        for sequence in (-1, 0x100000000):
            with self.subTest(sequence=sequence):
                with self.assertRaises(ValueError):
                    create_cmd_message(
                        XgoCommandType.MOVE,
                        sequence=sequence,
                    )

    def test_sequence_boundary_values(self):
        for sequence in (0, 0xFFFFFFFF):
            with self.subTest(sequence=sequence):
                msg = create_cmd_message(
                    XgoCommandType.MOVE,
                    sequence=sequence,
                )

                self.assertEqual(msg.sequence, sequence)
                self.assertTrue(validate_cmd_message(msg))

    def test_invalid_timestamp_rejected(self):
        with self.assertRaises(ValueError):
            create_cmd_message(
                XgoCommandType.MOVE,
                sequence=1,
                timestamp_ns=0,
            )


# ============================================================================
# Telemetry message tests
# ============================================================================

class TestTelemetryMessage(unittest.TestCase):
    """Test telemetry creation and validation."""

    def test_telemetry_creation(self):
        msg = create_telem_message(
            sequence=1,
            status_flags=XgoStatusFlags.READY,
            battery_pct=80.0,
        )

        self.assertEqual(msg.version, PROTOCOL_VERSION)
        self.assertEqual(msg.sequence, 1)

        self.assertTrue(validate_telem_message(msg))

    def test_invalid_battery_rejected(self):
        for battery in (-1.0, 100.1, 150.0):
            with self.subTest(battery=battery):
                with self.assertRaises(ValueError):
                    create_telem_message(
                        sequence=1,
                        battery_pct=battery,
                    )

    def test_battery_boundary_values(self):
        for battery in (0.0, 100.0):
            with self.subTest(battery=battery):
                msg = create_telem_message(
                    sequence=1,
                    battery_pct=battery,
                )

                self.assertTrue(validate_telem_message(msg))

    def test_nan_telemetry_rejected(self):
        msg = create_telem_message(sequence=1)

        msg.pitch_deg = math.nan

        self.assertFalse(validate_telem_message(msg))

    def test_infinite_telemetry_rejected(self):
        msg = create_telem_message(sequence=1)

        msg.roll_deg = math.inf

        self.assertFalse(validate_telem_message(msg))

    def test_invalid_status_flags_rejected(self):
        msg = create_telem_message(sequence=1)

        msg.status_flags = 1 << 15

        self.assertFalse(validate_telem_message(msg))

    def test_zero_timestamp_rejected(self):
        msg = create_telem_message(sequence=1)

        msg.timestamp_ns = 0

        self.assertFalse(validate_telem_message(msg))

    def test_invalid_telemetry_version(self):
        msg = create_telem_message(sequence=1)

        msg.version = 99

        self.assertFalse(validate_telem_message(msg))

    def test_invalid_sequence_rejected(self):
        for sequence in (-1, 0x100000000):
            with self.subTest(sequence=sequence):
                with self.assertRaises(ValueError):
                    create_telem_message(sequence=sequence)


# ============================================================================
# Serialization tests
# ============================================================================

class TestSerialization(unittest.TestCase):
    """Test encoding and decoding of binary messages."""

    def test_command_round_trip(self):
        original = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=42,
            vx=0.5,
            wz=0.3,
            height_z=90.0,
            action_id=0,
            timestamp_ns=123456789,
        )

        encoded = encode_cmd_message(original)

        self.assertEqual(len(encoded), CMD_MSG_SIZE)

        decoded = decode_cmd_message(encoded)

        self.assertEqual(decoded.sequence, original.sequence)
        self.assertEqual(decoded.version, original.version)
        self.assertEqual(decoded.command_type, original.command_type)
        self.assertEqual(decoded.vx, original.vx)
        self.assertEqual(decoded.wz, original.wz)
        self.assertEqual(decoded.height_z, original.height_z)
        self.assertEqual(decoded.action_id, original.action_id)
        self.assertEqual(decoded.timestamp_ns, original.timestamp_ns)

    def test_telemetry_round_trip(self):
        original = create_telem_message(
            sequence=42,
            status_flags=(
                XgoStatusFlags.READY
                | XgoStatusFlags.SENSOR_VALID
            ),
            pitch_deg=2.5,
            roll_deg=-1.0,
            yaw_deg=45.0,
            battery_pct=90.0,
            timestamp_ns=123456789,
        )

        encoded = encode_telem_message(original)

        self.assertEqual(len(encoded), TELEM_MSG_SIZE)

        decoded = decode_telem_message(encoded)

        self.assertEqual(decoded.sequence, original.sequence)
        self.assertEqual(decoded.version, original.version)
        self.assertEqual(decoded.status_flags, original.status_flags)
        self.assertEqual(decoded.pitch_deg, original.pitch_deg)
        self.assertEqual(decoded.roll_deg, original.roll_deg)
        self.assertEqual(decoded.yaw_deg, original.yaw_deg)
        self.assertEqual(decoded.battery_pct, original.battery_pct)
        self.assertEqual(decoded.timestamp_ns, original.timestamp_ns)

    def test_invalid_command_size(self):
        with self.assertRaises(ValueError):
            decode_cmd_message(b"\x00" * 10)

    def test_invalid_telemetry_size(self):
        with self.assertRaises(ValueError):
            decode_telem_message(b"\x00" * 10)

    def test_invalid_command_contents_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.version = 99

        with self.assertRaises(ValueError):
            encode_cmd_message(msg)

    def test_invalid_telemetry_contents_rejected(self):
        msg = create_telem_message(sequence=1)

        msg.battery_pct = 150.0

        with self.assertRaises(ValueError):
            encode_telem_message(msg)

    def test_invalid_decoded_command_rejected(self):
        msg = create_cmd_message(
            XgoCommandType.MOVE,
            sequence=1,
        )

        msg.version = 99

        with self.assertRaises(ValueError):
            decode_cmd_message(bytes(msg))

    def test_invalid_decoded_telemetry_rejected(self):
        msg = create_telem_message(sequence=1)

        msg.status_flags = 1 << 15

        with self.assertRaises(ValueError):
            decode_telem_message(bytes(msg))


# ============================================================================
# Message queue priority tests
# ============================================================================

class TestMessagePriority(unittest.TestCase):
    """Verify command priority assignments."""

    def test_estop_priority(self):
        msg = create_cmd_message(
            XgoCommandType.ESTOP,
            sequence=1,
        )

        self.assertEqual(
            cmd_mq_priority(msg),
            MQ_PRIORITY_EMERGENCY,
        )

    def test_normal_command_priority(self):
        normal_commands = (
            XgoCommandType.MOVE,
            XgoCommandType.HEIGHT,
            XgoCommandType.ACTION,
            XgoCommandType.SHUTDOWN,
            XgoCommandType.HEARTBEAT,
            XgoCommandType.RESET,
        )

        for command_type in normal_commands:
            with self.subTest(command_type=command_type):
                msg = create_cmd_message(
                    command_type,
                    sequence=1,
                    action_id=1 if command_type == XgoCommandType.ACTION else 0,
                )

                self.assertEqual(
                    cmd_mq_priority(msg),
                    MQ_PRIORITY_NORMAL,
                )

    def test_estop_priority_is_higher(self):
        self.assertGreater(
            MQ_PRIORITY_EMERGENCY,
            MQ_PRIORITY_NORMAL,
        )

    def test_priority_rejects_wrong_message_type(self):
        with self.assertRaises(TypeError):
            cmd_mq_priority(XgoTelemMsg())


if __name__ == "__main__":
    unittest.main()
