
"""
ipc_bridge.py

Python-side POSIX message queue interface for the XGO Rider 2.

Responsibilities:
    - Open the POSIX message queues created by the C supervisor.
    - Receive and decode command messages.
    - Validate command freshness and sequence numbers.
    - Enforce the bridge communication interlock.
    - Encode and publish telemetry messages.
    - Monitor the C supervisor heartbeat.

Does NOT:
    - Create or unlink POSIX message queues.
    - Communicate with the UART directly.
    - Import or call xgolib.
    - Implement robot-specific safety limits.
    - Execute robot commands.
"""

from __future__ import annotations

import logging
import time

from dataclasses import dataclass
from typing import Optional

import posix_ipc

from .ipc_protocol import (
    XgoCmdMsg,
    XgoTelemMsg,
    XgoCommandType,
    CMD_MSG_SIZE,
    TELEM_MSG_SIZE,
    MQ_PRIORITY_NORMAL,
    cmd_mq_priority,
    decode_cmd_message,
    encode_telem_message,
)


# ============================================================================
# Module configuration
# ============================================================================

CMD_QUEUE_NAME = "/xgo_cmd_mq"
TELEM_QUEUE_NAME = "/xgo_telem_mq"

UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000


logger = logging.getLogger(__name__)


# ============================================================================
# Configuration
# ============================================================================

@dataclass(frozen=True, slots=True)
class BridgeConfig:
    """
    Immutable configuration for the Python IPC bridge.

    Timeout values must be selected and verified during integration.
    """

    receive_timeout_s: float
    supervisor_timeout_s: float
    startup_grace_s: float
    max_command_age_s: float
    telemetry_send_timeout_s: float

    def __post_init__(self) -> None:
        values = (
            self.receive_timeout_s,
            self.supervisor_timeout_s,
            self.startup_grace_s,
            self.max_command_age_s,
            self.telemetry_send_timeout_s,
        )

        if any(not isinstance(value, (int, float)) for value in values):
            raise TypeError("Timeout values must be numeric")

        if any(not 0 < value < float("inf") for value in values):
            raise ValueError("Timeout values must be finite and positive")


# ============================================================================
# Command sequence tracking
# ============================================================================

class CommandSequenceTracker:
    """
    Tracks command sequence numbers.

    Supports uint32_t wraparound.

    A sequence number is accepted only if it is newer than the
    previously accepted sequence number.
    """

    def __init__(self) -> None:
        self._last_sequence: Optional[int] = None

    def accept(self, sequence: int) -> bool:
        """
        Return True if the sequence number is new.

        The first sequence number is accepted.

        Subsequent values must be newer according to modular
        uint32_t sequence comparison.
        """

        if not 0 <= sequence <= UINT32_MASK:
            return False

        if self._last_sequence is None:
            self._last_sequence = sequence
            return True

        difference = (
            sequence - self._last_sequence
        ) & UINT32_MASK

        if not 0 < difference < UINT32_HALF_RANGE:
            return False

        self._last_sequence = sequence
        return True

    @property
    def last_sequence(self) -> Optional[int]:
        return self._last_sequence


# ============================================================================
# POSIX IPC bridge
# ============================================================================

class IpcBridge:
    """
    Owns the Python-side handles to both POSIX message queues.

    The C supervisor is responsible for queue creation and removal.
    """

    def __init__(self, config: BridgeConfig) -> None:

        self._config = config

        self._cmd_queue: Optional[posix_ipc.MessageQueue] = None
        self._telem_queue: Optional[posix_ipc.MessageQueue] = None

        self._sequence_tracker = CommandSequenceTracker()
        self._emergency_sequence = CommandSequenceTracker()
        self._last_estop_time_ns = 0

        self._start_time_ns = time.monotonic_ns()
        self._last_supervisor_heartbeat_ns: Optional[int] = None

        self._supervisor_timeout_latched = False
        self._emergency_latched = False
        self._shutdown_requested = False

    # ========================================================================
    # Queue lifecycle
    # ========================================================================

    def open(self) -> None:
        """
        Open existing message queues.

        The C supervisor must create them before starting this process.

        Python opens:
            /xgo_cmd_mq   -> read-only
            /xgo_telem_mq -> write-only

        Raises:
            OSError: If queues cannot be opened.
            RuntimeError: If queue attributes are incompatible.
        """

        if self._cmd_queue is not None or self._telem_queue is not None:
            raise RuntimeError("IPC bridge is already open")

        try:
            self._cmd_queue = posix_ipc.MessageQueue(
                CMD_QUEUE_NAME,
                read=True,
                write=False,
            )

            self._telem_queue = posix_ipc.MessageQueue(
                TELEM_QUEUE_NAME,
                read=False,
                write=True,
            )

            self._verify_queue_attributes()

        except Exception:
            self.close()
            raise

        logger.info("POSIX message queues opened successfully")

    def _verify_queue_attributes(self) -> None:
        """Verify that both queues use the expected message size."""

        if self._cmd_queue is None or self._telem_queue is None:
            raise RuntimeError("Message queues are not open")

        if self._cmd_queue.max_message_size != CMD_MSG_SIZE:
            raise RuntimeError(
                "Invalid command MQ message size: "
                f"expected {CMD_MSG_SIZE}, "
                f"got {self._cmd_queue.max_message_size}"
            )

        if self._telem_queue.max_message_size != TELEM_MSG_SIZE:
            raise RuntimeError(
                "Invalid telemetry MQ message size: "
                f"expected {TELEM_MSG_SIZE}, "
                f"got {self._telem_queue.max_message_size}"
            )

    def close(self) -> None:
        """
        Close Python's queue descriptors.

        Does NOT unlink the queues. Their lifecycle belongs to C.
        """

        if self._cmd_queue is not None:
            self._cmd_queue.close()
            self._cmd_queue = None

        if self._telem_queue is not None:
            self._telem_queue.close()
            self._telem_queue = None

        logger.info("POSIX message queues closed")

    # ========================================================================
    # Command reception
    # ========================================================================

    def receive_command(self) -> Optional[XgoCmdMsg]:
        """
        Receive and validate one command.

        Returns:
            XgoCmdMsg: A valid command for the hardware/application layer.
            None: No command available or command rejected.

        HEARTBEAT messages update supervisor liveness but are never
        returned as movement commands.
        """

        if self._cmd_queue is None:
            raise RuntimeError("Command queue is not open")

        try:
            data, priority = self._cmd_queue.receive(
                timeout=self._config.receive_timeout_s
            )

        except posix_ipc.BusyError:
            return None

        except posix_ipc.SignalError:
            # An interrupted receive is not a valid command.
            return None

        try:
            command = decode_cmd_message(data)

        except ValueError as exc:
            logger.warning("Rejected invalid command: %s", exc)
            return None

        if not self._is_command_fresh(command):
            logger.warning(
                "Rejected stale command, sequence=%u",
                command.sequence,
            )
            return None

        if priority != cmd_mq_priority(command):
            logger.warning("Rejected command with unexpected MQ priority")
            return None

        # Priority can reorder ESTOP ahead of earlier normal traffic.
        if command.command_type == XgoCommandType.ESTOP:
            if not self._emergency_sequence.accept(command.sequence):
                return None
            self._last_estop_time_ns = max(
                self._last_estop_time_ns, command.timestamp_ns
            )
            self._latch_emergency()
            return command

        if not self._sequence_tracker.accept(command.sequence):
            logger.warning(
                "Rejected duplicate/out-of-order command, sequence=%u",
                command.sequence,
            )
            return None

        self._update_supervisor_liveness(command)

        if command.command_type == XgoCommandType.HEARTBEAT:
            return None

        if command.command_type == XgoCommandType.SHUTDOWN:
            self._shutdown_requested = True
            return command

        if command.timestamp_ns <= self._last_estop_time_ns:
            return None

        if self._emergency_latched:
            if command.command_type == XgoCommandType.RESET:
                return command

            logger.warning(
                "Rejected command while emergency is latched"
            )
            return None

        return command

    # ========================================================================
    # Command validation helpers
    # ========================================================================

    def _is_command_fresh(self, command: XgoCmdMsg) -> bool:
        """
        Check command age using the shared monotonic clock domain.

        Both C and Python execute on the same Raspberry Pi.
        """

        now_ns = time.monotonic_ns()

        age_ns = now_ns - command.timestamp_ns

        if age_ns < 0:
            return False

        max_age_ns = int(
            self._config.max_command_age_s * 1_000_000_000
        )

        return age_ns <= max_age_ns

    def _update_supervisor_liveness(self, command: XgoCmdMsg) -> None:
        """
        Refresh supervisor liveness only upon receiving an explicit
        HEARTBEAT message.

        Normal commands must not substitute for supervisor heartbeats.
        """

        if command.command_type != XgoCommandType.HEARTBEAT:
            return

        if not self.supervisor_alive():
            return

        if (self._last_supervisor_heartbeat_ns is None or
                command.timestamp_ns > self._last_supervisor_heartbeat_ns):
            self._last_supervisor_heartbeat_ns = command.timestamp_ns

    # ========================================================================
    # Telemetry transmission
    # ========================================================================

    def send_telemetry(self, telemetry: XgoTelemMsg) -> bool:
        """
        Encode and send telemetry to the C supervisor.

        Returns:
            True: Message successfully queued.
            False: Queue was full before the timeout expired.

        Telemetry is a latest-state stream, so a failed transmission
        does not block the hardware processing loop indefinitely.
        """

        if self._telem_queue is None:
            raise RuntimeError("Telemetry queue is not open")

        data = encode_telem_message(telemetry)

        try:
            self._telem_queue.send(
                data,
                timeout=self._config.telemetry_send_timeout_s,
                priority=MQ_PRIORITY_NORMAL,
            )

        except posix_ipc.BusyError:
            logger.warning("Telemetry MQ is full; sample dropped")
            return False

        except posix_ipc.SignalError:
            logger.warning("Telemetry transmission interrupted")
            return False

        return True

    # ========================================================================
    # Supervisor watchdog
    # ========================================================================

    def supervisor_alive(self) -> bool:
        """
        Check whether the C supervisor heartbeat is still fresh.

        During startup, a configurable grace period is allowed.

        Once a timeout occurs, the communication fault remains latched.
        A subsequent heartbeat does not automatically restore operation.
        """

        if self._supervisor_timeout_latched:
            return False

        now_ns = time.monotonic_ns()

        if self._last_supervisor_heartbeat_ns is None:

            startup_elapsed_ns = now_ns - self._start_time_ns

            startup_grace_ns = int(
                self._config.startup_grace_s * 1_000_000_000
            )

            if startup_elapsed_ns <= startup_grace_ns:
                return True

            self._latch_supervisor_timeout()
            return False

        heartbeat_age_ns = (
            now_ns - self._last_supervisor_heartbeat_ns
        )

        timeout_ns = int(
            self._config.supervisor_timeout_s * 1_000_000_000
        )

        if heartbeat_age_ns > timeout_ns:
            self._latch_supervisor_timeout()
            return False

        return True

    def _latch_supervisor_timeout(self) -> None:
        """Latch a supervisor communication failure exactly once."""

        if self._supervisor_timeout_latched:
            return

        self._supervisor_timeout_latched = True
        self._latch_emergency()

        logger.critical(
            "C supervisor heartbeat timeout detected"
        )

    # ========================================================================
    # Emergency and shutdown state
    # ========================================================================

    def _latch_emergency(self) -> None:
        """Latch emergency state without performing hardware operations."""

        if self._emergency_latched:
            return

        self._emergency_latched = True

        logger.critical("Bridge emergency state latched")

    def complete_recovery(self) -> None:
        """
        Clear the bridge emergency latch after verified recovery.

        This must only be called by the application after an explicit
        recovery procedure has succeeded.

        A supervisor communication timeout cannot be cleared this way.
        """

        if self._supervisor_timeout_latched:
            raise RuntimeError(
                "Cannot recover while supervisor communication is lost"
            )

        self._emergency_latched = False

        logger.info("Bridge emergency state cleared")

    @property
    def emergency_active(self) -> bool:
        return self._emergency_latched

    @property
    def shutdown_requested(self) -> bool:
        return self._shutdown_requested

    @property
    def watchdog_expired(self) -> bool:
        return self._supervisor_timeout_latched

    @property
    def supervisor_heartbeat_received(self) -> bool:
        return self._last_supervisor_heartbeat_ns is not None

