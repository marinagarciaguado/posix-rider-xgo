
"""
ipc_protocol.py

Binary communication protocol between the XGO Rider 2 C supervisor
and the Python xgolib bridge.

Responsibilities:
    - Define protocol constants and message structures.
    - Construct command and telemetry messages.
    - Validate message contents.
    - Serialize and deserialize messages.

This module does NOT:
    - Manage POSIX message queues.
    - Handle threads or synchronization.
    - Communicate with the robot hardware.
    - Implement application-level safety policies.
"""

from __future__ import annotations

import ctypes
import math
import time

from enum import IntEnum, IntFlag


# ============================================================================
# Protocol configuration
# ============================================================================

PROTOCOL_VERSION = 1

CMD_MSG_SIZE = 32
TELEM_MSG_SIZE = 32


# ============================================================================
# Message queue priorities
# ============================================================================

MQ_PRIORITY_NORMAL = 1
MQ_PRIORITY_EMERGENCY = 2


# ============================================================================
# Command opcodes (C -> Python)
# ============================================================================

class XgoCommandType(IntEnum):
    """Commands sent from the C supervisor to the Python bridge."""

    MOVE = 1
    HEIGHT = 2
    ACTION = 3
    ESTOP = 4
    SHUTDOWN = 5
    HEARTBEAT = 6
    RESET = 7


# ============================================================================
# Telemetry status flags (Python -> C)
# ============================================================================

class XgoStatusFlags(IntFlag):
    """Status flags reported by the Python bridge."""

    READY = 1 << 0
    SENSOR_VALID = 1 << 1
    EMERGENCY_ACTIVE = 1 << 2
    BRIDGE_ERROR = 1 << 3


# ============================================================================
# Internal protocol constants
# ============================================================================

_KNOWN_COMMANDS = frozenset(XgoCommandType)

_ALLOWED_STATUS_FLAGS = (
    XgoStatusFlags.READY
    | XgoStatusFlags.SENSOR_VALID
    | XgoStatusFlags.EMERGENCY_ACTIVE
    | XgoStatusFlags.BRIDGE_ERROR
)


# ============================================================================
# Message structures
# ============================================================================

class XgoCmdMsg(ctypes.LittleEndianStructure):
    """
    Command message.

    Direction:
        C supervisor -> Python bridge

    Expected size: 32 bytes.
    """

    _fields_ = [
        ("sequence", ctypes.c_uint32),
        ("version", ctypes.c_uint16),
        ("command_type", ctypes.c_uint16),

        ("vx", ctypes.c_float),
        ("wz", ctypes.c_float),
        ("height_z", ctypes.c_float),

        ("action_id", ctypes.c_uint32),

        ("timestamp_ns", ctypes.c_uint64),
    ]


class XgoTelemMsg(ctypes.LittleEndianStructure):
    """
    Telemetry message.

    Direction:
        Python bridge -> C supervisor

    Expected size: 32 bytes.
    """

    _fields_ = [
        ("sequence", ctypes.c_uint32),
        ("version", ctypes.c_uint16),
        ("status_flags", ctypes.c_uint16),

        ("pitch_deg", ctypes.c_float),
        ("roll_deg", ctypes.c_float),
        ("yaw_deg", ctypes.c_float),
        ("battery_pct", ctypes.c_float),

        ("timestamp_ns", ctypes.c_uint64),
    ]


# ============================================================================
# Compile-time equivalent: Python protocol layout verification
# ============================================================================

def _verify_protocol_layout() -> None:
    """
    Verify that Python structures match the C protocol layout.

    Raises RuntimeError if the ABI differs from the expected layout.
    """

    if ctypes.sizeof(XgoCmdMsg) != CMD_MSG_SIZE:
        raise RuntimeError(
            f"Invalid XgoCmdMsg size: {ctypes.sizeof(XgoCmdMsg)}"
        )

    if ctypes.sizeof(XgoTelemMsg) != TELEM_MSG_SIZE:
        raise RuntimeError(
            f"Invalid XgoTelemMsg size: {ctypes.sizeof(XgoTelemMsg)}"
        )

    expected_cmd_offsets = {
        "sequence": 0,
        "version": 4,
        "command_type": 6,
        "vx": 8,
        "wz": 12,
        "height_z": 16,
        "action_id": 20,
        "timestamp_ns": 24,
    }

    expected_telem_offsets = {
        "sequence": 0,
        "version": 4,
        "status_flags": 6,
        "pitch_deg": 8,
        "roll_deg": 12,
        "yaw_deg": 16,
        "battery_pct": 20,
        "timestamp_ns": 24,
    }

    for field, expected_offset in expected_cmd_offsets.items():
        actual_offset = getattr(XgoCmdMsg, field).offset

        if actual_offset != expected_offset:
            raise RuntimeError(
                f"Invalid XgoCmdMsg.{field} offset: "
                f"expected {expected_offset}, got {actual_offset}"
            )

    for field, expected_offset in expected_telem_offsets.items():
        actual_offset = getattr(XgoTelemMsg, field).offset

        if actual_offset != expected_offset:
            raise RuntimeError(
                f"Invalid XgoTelemMsg.{field} offset: "
                f"expected {expected_offset}, got {actual_offset}"
            )


_verify_protocol_layout()


# ============================================================================
# Timestamp utilities
# ============================================================================

def monotonic_now_ns() -> int:
    """
    Return the current monotonic timestamp in nanoseconds.

    Uses the same clock domain as clock_gettime(CLOCK_MONOTONIC)
    in the C supervisor.
    """

    return time.monotonic_ns()


# ============================================================================
# Message construction
# ============================================================================

def create_cmd_message(
    command_type: XgoCommandType,
    sequence: int,
    *,
    vx: float = 0.0,
    wz: float = 0.0,
    height_z: float = 0.0,
    action_id: int = 0,
    timestamp_ns: int | None = None,
) -> XgoCmdMsg:
    """
    Create an initialized command message.

    If timestamp_ns is omitted, the current monotonic timestamp
    is used.

    Raises:
        ValueError: If arguments cannot be represented by the protocol.
    """

    try:
        command = XgoCommandType(command_type)
    except ValueError as exc:
        raise ValueError(
            f"Unknown command type: {command_type}"
        ) from exc

    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit in uint32_t")

    if not 0 <= action_id <= 0xFFFFFFFF:
        raise ValueError("action_id must fit in uint32_t")

    if timestamp_ns is None:
        timestamp_ns = monotonic_now_ns()

    if not 0 < timestamp_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("timestamp_ns must fit in uint64_t and be nonzero")

    message = XgoCmdMsg()

    message.sequence = sequence
    message.version = PROTOCOL_VERSION
    message.command_type = command

    message.vx = vx
    message.wz = wz
    message.height_z = height_z

    message.action_id = action_id
    message.timestamp_ns = timestamp_ns

    if not validate_cmd_message(message):
        raise ValueError("Invalid command message")

    return message


def create_telem_message(
    sequence: int,
    *,
    status_flags: XgoStatusFlags = XgoStatusFlags(0),
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    yaw_deg: float = 0.0,
    battery_pct: float = 0.0,
    timestamp_ns: int | None = None,
) -> XgoTelemMsg:
    """
    Create an initialized telemetry message.

    If timestamp_ns is omitted, the current monotonic timestamp
    is used.

    Raises:
        ValueError: If arguments cannot be represented by the protocol.
    """

    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence must fit in uint32_t")

    if timestamp_ns is None:
        timestamp_ns = monotonic_now_ns()

    if not 0 < timestamp_ns <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("timestamp_ns must fit in uint64_t and be nonzero")

    message = XgoTelemMsg()

    message.sequence = sequence
    message.version = PROTOCOL_VERSION
    message.status_flags = int(status_flags)

    message.pitch_deg = pitch_deg
    message.roll_deg = roll_deg
    message.yaw_deg = yaw_deg
    message.battery_pct = battery_pct

    message.timestamp_ns = timestamp_ns

    if not validate_telem_message(message):
        raise ValueError("Invalid telemetry message")

    return message


# ============================================================================
# Message validation
# ============================================================================

def validate_cmd_message(message: XgoCmdMsg) -> bool:
    """
    Validate a command message.

    Checks:
        - Correct structure type.
        - Supported protocol version.
        - Recognized command opcode.
        - Finite floating-point values.
        - Valid action identifier for ACTION commands.
        - Nonzero timestamp.

    Does not enforce robot-specific safety limits.
    """

    if not isinstance(message, XgoCmdMsg):
        return False

    if message.version != PROTOCOL_VERSION:
        return False

    if message.command_type not in _KNOWN_COMMANDS:
        return False

    if message.timestamp_ns == 0:
        return False

    if not all(math.isfinite(value) for value in (
        message.vx,
        message.wz,
        message.height_z,
    )):
        return False

    if message.command_type == XgoCommandType.ACTION:
        if message.action_id == 0:
            return False

    return True


def validate_telem_message(message: XgoTelemMsg) -> bool:
    """
    Validate a telemetry message.

    Checks:
        - Correct structure type.
        - Supported protocol version.
        - Recognized status flags.
        - Finite sensor values.
        - Battery percentage within [0, 100].
        - Nonzero timestamp.
    """

    if not isinstance(message, XgoTelemMsg):
        return False

    if message.version != PROTOCOL_VERSION:
        return False

    if message.timestamp_ns == 0:
        return False

    unknown_flags = (
        message.status_flags & ~int(_ALLOWED_STATUS_FLAGS)
    )

    if unknown_flags != 0:
        return False

    if not all(math.isfinite(value) for value in (
        message.pitch_deg,
        message.roll_deg,
        message.yaw_deg,
        message.battery_pct,
    )):
        return False

    if not 0.0 <= message.battery_pct <= 100.0:
        return False

    return True


# ============================================================================
# Serialization
# ============================================================================

def encode_cmd_message(message: XgoCmdMsg) -> bytes:
    """
    Serialize a validated command message into 32 bytes.

    The resulting bytes can be sent directly through a POSIX MQ.
    """

    if not validate_cmd_message(message):
        raise ValueError("Cannot encode invalid command message")

    return bytes(message)


def encode_telem_message(message: XgoTelemMsg) -> bytes:
    """
    Serialize a validated telemetry message into 32 bytes.
    """

    if not validate_telem_message(message):
        raise ValueError("Cannot encode invalid telemetry message")

    return bytes(message)


# ============================================================================
# Deserialization
# ============================================================================

def decode_cmd_message(data: bytes) -> XgoCmdMsg:
    """
    Deserialize and validate a 32-byte command message.

    Raises:
        ValueError: If the message has an invalid size or contents.
    """

    if len(data) != CMD_MSG_SIZE:
        raise ValueError(
            f"Invalid command message size: "
            f"expected {CMD_MSG_SIZE}, got {len(data)}"
        )

    message = XgoCmdMsg.from_buffer_copy(data)

    if not validate_cmd_message(message):
        raise ValueError("Received invalid command message")

    return message


def decode_telem_message(data: bytes) -> XgoTelemMsg:
    """
    Deserialize and validate a 32-byte telemetry message.
    """

    if len(data) != TELEM_MSG_SIZE:
        raise ValueError(
            f"Invalid telemetry message size: "
            f"expected {TELEM_MSG_SIZE}, got {len(data)}"
        )

    message = XgoTelemMsg.from_buffer_copy(data)

    if not validate_telem_message(message):
        raise ValueError("Received invalid telemetry message")

    return message


# ============================================================================
# Message queue utilities
# ============================================================================

def cmd_mq_priority(message: XgoCmdMsg) -> int:
    """
    Return the POSIX MQ priority for a command.

    ESTOP receives higher priority than normal commands.
    """

    if not isinstance(message, XgoCmdMsg):
        raise TypeError("Expected XgoCmdMsg")

    if message.command_type == XgoCommandType.ESTOP:
        return MQ_PRIORITY_EMERGENCY

    return MQ_PRIORITY_NORMAL
