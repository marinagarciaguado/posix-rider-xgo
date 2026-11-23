
"""
xgo_bridge.py

Main Python child process for the XGO Rider 2 supervisory architecture.

Responsibilities:
    - Manage application lifecycle.
    - Initialize hardware and IPC.
    - Publish startup readiness.
    - Dispatch validated commands.
    - Poll and publish robot telemetry.
    - Monitor supervisor liveness.
    - Manage emergency and recovery states.
    - Shut down hardware and IPC resources.
"""

from __future__ import annotations

import logging
import math
import os
import signal
import sys
import time

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from .ipc_bridge import BridgeConfig, IpcBridge

from .ipc_protocol import (
    XgoCmdMsg,
    XgoCommandType,
    XgoStatusFlags,
    create_telem_message,
)

from .hardware.xgo_adapter import (
    RobotTelemetry,
    XgoHardware,
    XgolibRiderAdapter,
)


logger = logging.getLogger("xgo_bridge")


# ============================================================================
# Logging
# ============================================================================

def configure_logging() -> None:

    logging.basicConfig(
        level=logging.INFO,
        format=(
            "%(asctime)s.%(msecs)03d "
            "[%(levelname)s] "
            "%(name)s: %(message)s"
        ),
        datefmt="%Y-%m-%d %H:%M:%S",
    )


# ============================================================================
# Application state machine
# ============================================================================

class BridgeState(Enum):
    STARTING = auto()
    NORMAL = auto()
    EMERGENCY = auto()
    RECOVERY = auto()
    SHUTDOWN = auto()


# ============================================================================
# Runtime configuration
# ============================================================================

@dataclass(frozen=True, slots=True)
class BridgeSettings:

    ipc: BridgeConfig

    telemetry_period_s: float
    telemetry_max_age_s: float

    recovery_max_pitch_deg: float
    recovery_max_roll_deg: float
    recovery_min_battery_pct: float

    def __post_init__(self) -> None:

        values = (
            self.telemetry_period_s,
            self.telemetry_max_age_s,
            self.recovery_max_pitch_deg,
            self.recovery_max_roll_deg,
        )

        if any(
            not math.isfinite(value) or value <= 0
            for value in values
        ):
            raise ValueError("Invalid bridge settings")

        if not 0.0 <= self.recovery_min_battery_pct <= 100.0:
            raise ValueError(
                "Invalid recovery battery threshold"
            )


# ============================================================================
# Application
# ============================================================================

class XgoBridgeApplication:

    def __init__(
        self,
        settings: BridgeSettings,
        hardware: XgoHardware,
        *,
        telemetry_only: bool = False,
        supervisor_pid: int | None = None,
    ) -> None:

        self._settings = settings
        self._hardware = hardware
        self._telemetry_only = telemetry_only
        self._supervisor_pid = supervisor_pid

        self._ipc = IpcBridge(settings.ipc)

        self._state = BridgeState.STARTING

        self._running = False
        self._hardware_connected = False
        self._ipc_open = False
        self._start_attempted = False

        self._telemetry_sequence = 0

        self._last_telemetry: Optional[RobotTelemetry] = None
        self._last_telemetry_time_ns: Optional[int] = None

        self._last_telemetry_attempt_ns: Optional[int] = None
        self._last_motion_time_ns: Optional[int] = None
        self._stop_failed = False
        self.shutdown_failed = False

        self._shutdown_requested = False

    # ========================================================================
    # Lifecycle
    # ========================================================================

    def start(self) -> None:

        if self._start_attempted:
            raise RuntimeError(
                "Application has already been started"
            )

        self._start_attempted = True

        logger.info("Starting XGO bridge")

        try:
            self._ipc.open()
            self._ipc_open = True

            self._hardware.connect()
            self._hardware_connected = True

            self._refresh_telemetry()

            if self._last_telemetry is None:
                raise RuntimeError(
                    "Initial telemetry unavailable"
                )

            if not self._publish_telemetry(
                extra_flags=XgoStatusFlags.READY
            ):
                raise RuntimeError(
                    "Failed to publish READY telemetry"
                )

            self._running = not self._shutdown_requested

            logger.info(
                "Hardware initialized; READY telemetry published"
            )

        except Exception:
            logger.exception("Bridge initialization failed")
            self.close()
            raise

    def run(self) -> None:

        if self._shutdown_requested:
            self.close()
            return

        if not self._running:
            raise RuntimeError(
                "Application has not been started"
            )

        try:
            self._wait_for_supervisor()

            while self._running:

                self._check_parent()
                self._check_supervisor_watchdog()

                if not self._running:
                    break

                command = self._ipc.receive_command()
                if not self._running:
                    break

                self._check_parent()
                self._check_supervisor_watchdog()
                self._check_telemetry_freshness()
                self._check_motion_expiry()

                if command is not None:
                    self._dispatch_command(command)

                if not self._running:
                    break

                self._poll_telemetry_if_due()

        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")

        except Exception:
            logger.exception(
                "Fatal bridge application error"
            )

            self._enter_emergency(
                "Application failure"
            )
            raise

        finally:
            self.close()

    def request_shutdown(self) -> None:

        self._shutdown_requested = True
        self._running = False

    def close(self) -> None:

        if self._state == BridgeState.SHUTDOWN:
            return

        logger.info("Shutting down XGO bridge")

        self._running = False
        self._state = BridgeState.SHUTDOWN

        if self._hardware_connected:

            try:
                self._request_zero_velocity()

            except Exception:
                self.shutdown_failed = True
                logger.exception(
                    "Failed to request zero-velocity response"
                )

            try:
                self._hardware.close()

            except Exception:
                self.shutdown_failed = True
                logger.exception(
                    "Hardware cleanup failed"
                )

            self._hardware_connected = False

        if self._ipc_open:

            try:
                self._ipc.close()

            except Exception:
                self.shutdown_failed = True
                logger.exception(
                    "IPC cleanup failed"
                )

            self._ipc_open = False

        logger.info("XGO bridge shutdown complete")

    # ========================================================================
    # Startup handshake
    # ========================================================================

    def _wait_for_supervisor(self) -> None:

        logger.info(
            "Waiting for C supervisor heartbeat"
        )

        while self._running:

            self._check_parent()
            if self._ipc.supervisor_heartbeat_received:

                if not self._ipc.supervisor_alive():
                    raise RuntimeError(
                        "Supervisor watchdog expired during startup"
                    )

                if self._state == BridgeState.STARTING:
                    self._state = BridgeState.NORMAL
                logger.info("Supervisor connected; state=%s", self._state.name)

                return

            if not self._ipc.supervisor_alive():
                raise TimeoutError(
                    "Supervisor startup heartbeat timeout"
                )

            command = self._ipc.receive_command()

            if self._running and command is not None:
                self._dispatch_command(command)

    # ========================================================================
    # Command dispatch
    # ========================================================================

    def _dispatch_command(self, command: XgoCmdMsg) -> None:

        if self._state == BridgeState.SHUTDOWN:
            return

        command_type = XgoCommandType(
            command.command_type
        )

        logger.debug(
            "Received command type=%s sequence=%d",
            command_type.name,
            command.sequence,
        )

        if command_type == XgoCommandType.ESTOP:
            self._enter_emergency(
                "ESTOP command received"
            )
            return

        if command_type == XgoCommandType.SHUTDOWN:
            self.request_shutdown()
            return

        if command_type == XgoCommandType.RESET:
            self._attempt_recovery()
            return

        if self._state != BridgeState.NORMAL:

            logger.warning(
                "Rejected %s while state is %s",
                command_type.name,
                self._state.name,
            )

            return

        if self._telemetry_only:
            return

        try:

            if command_type == XgoCommandType.MOVE:

                self._hardware.move(
                    vx=command.vx,
                    wz=command.wz,
                )
                self._last_motion_time_ns = (
                    command.timestamp_ns if command.vx or command.wz else None
                )

            elif command_type == XgoCommandType.HEIGHT:

                self._hardware.set_height(
                    command.height_z
                )

            elif command_type == XgoCommandType.ACTION:

                self._hardware.perform_action(
                    command.action_id
                )

            elif command_type == XgoCommandType.HEARTBEAT:

                # HEARTBEAT is consumed by IpcBridge.
                # It must never renew a movement target.
                return

            else:
                logger.warning(
                    "Unsupported command: %s",
                    command_type,
                )

        except Exception:

            logger.exception(
                "Hardware command execution failed"
            )

            self._enter_emergency(
                "Hardware command execution failure"
            )

    # ========================================================================
    # Emergency handling
    # ========================================================================

    def _enter_emergency(self, reason: str) -> None:

        if self._state in (
            BridgeState.EMERGENCY,
            BridgeState.SHUTDOWN,
        ):
            return

        logger.critical(
            "Entering EMERGENCY: %s",
            reason,
        )

        self._state = BridgeState.EMERGENCY
        self._last_motion_time_ns = None

        if self._hardware_connected:

            try:
                self._request_zero_velocity()

            except Exception:
                self._stop_failed = True
                logger.exception("Emergency hardware response failed")

    def _request_zero_velocity(self) -> None:
        if not self._telemetry_only:
            self._hardware.request_safe_response()

    def _check_motion_expiry(self) -> None:
        if self._state != BridgeState.NORMAL or self._last_motion_time_ns is None:
            return
        age_ns = time.monotonic_ns() - self._last_motion_time_ns
        if age_ns <= int(self._settings.ipc.max_command_age_s * 1_000_000_000):
            return
        self._last_motion_time_ns = None
        try:
            self._request_zero_velocity()
        except Exception:
            self._enter_emergency("Motion expired and zero velocity request failed")

    # ========================================================================
    # Recovery
    # ========================================================================

    def _attempt_recovery(self) -> None:

        if self._state != BridgeState.EMERGENCY:
            logger.warning(
                "RESET ignored outside EMERGENCY state"
            )
            return

        if not self._ipc.supervisor_alive():
            logger.error(
                "Recovery rejected: supervisor unavailable"
            )
            return

        if self._stop_failed:
            logger.error("Recovery rejected: emergency hardware response failed")
            return

        if not self._telemetry_is_fresh():
            logger.error(
                "Recovery rejected: telemetry is stale"
            )
            return

        telemetry = self._last_telemetry

        if telemetry is None:
            return

        if abs(telemetry.pitch_deg) > (
            self._settings.recovery_max_pitch_deg
        ):
            logger.error(
                "Recovery rejected: pitch outside safe range"
            )
            return

        if abs(telemetry.roll_deg) > (
            self._settings.recovery_max_roll_deg
        ):
            logger.error(
                "Recovery rejected: roll outside safe range"
            )
            return

        if telemetry.battery_pct < (
            self._settings.recovery_min_battery_pct
        ):
            logger.error(
                "Recovery rejected: battery too low"
            )
            return

        self._state = BridgeState.RECOVERY

        try:
            self._ipc.complete_recovery()

        except Exception:
            self._state = BridgeState.EMERGENCY

            logger.exception(
                "Failed to clear IPC emergency latch"
            )

            return

        self._state = BridgeState.NORMAL

        logger.warning(
            "Recovery conditions satisfied; "
            "NORMAL state restored"
        )

    # ========================================================================
    # Supervisor watchdog
    # ========================================================================

    def _check_supervisor_watchdog(self) -> None:

        if not self._ipc.supervisor_alive():

            self._enter_emergency(
                "C supervisor heartbeat timeout"
            )

    def _check_parent(self) -> None:
        if self._supervisor_pid is not None and os.getppid() != self._supervisor_pid:
            raise RuntimeError("C supervisor process disappeared")

    # ========================================================================
    # Telemetry
    # ========================================================================

    def _refresh_telemetry(self) -> None:

        self._last_telemetry_attempt_ns = (
            time.monotonic_ns()
        )

        telemetry = self._hardware.read_telemetry()

        self._last_telemetry = telemetry

        self._last_telemetry_time_ns = (
            time.monotonic_ns()
        )

    def _poll_telemetry_if_due(self) -> None:

        now_ns = time.monotonic_ns()

        if self._last_telemetry_attempt_ns is None:
            self._refresh_telemetry()
            self._publish_telemetry()
            return

        period_ns = int(
            self._settings.telemetry_period_s
            * 1_000_000_000
        )

        if (
            now_ns - self._last_telemetry_attempt_ns
            < period_ns
        ):
            return

        try:
            self._refresh_telemetry()
            self._publish_telemetry()

        except Exception:
            self._last_telemetry_time_ns = None
            logger.exception(
                "Telemetry acquisition failed"
            )

            self._enter_emergency(
                "Invalid or unavailable robot telemetry"
            )

            self._publish_telemetry(
                extra_flags=XgoStatusFlags.BRIDGE_ERROR,
                sensor_valid=False,
            )

    def _publish_telemetry(
        self,
        extra_flags: XgoStatusFlags = XgoStatusFlags(0),
        *,
        sensor_valid: bool = True,
    ) -> bool:

        if self._last_telemetry is None:
            return False

        flags = extra_flags

        if sensor_valid and self._last_telemetry_time_ns is not None:
            flags |= XgoStatusFlags.SENSOR_VALID

        if self._stop_failed:
            flags |= XgoStatusFlags.BRIDGE_ERROR

        if self._state == BridgeState.NORMAL:
            flags |= XgoStatusFlags.READY

        if self._state == BridgeState.EMERGENCY:
            flags |= XgoStatusFlags.EMERGENCY_ACTIVE

        telemetry = create_telem_message(
            sequence=self._telemetry_sequence,
            status_flags=flags,
            pitch_deg=self._last_telemetry.pitch_deg,
            roll_deg=self._last_telemetry.roll_deg,
            yaw_deg=self._last_telemetry.yaw_deg,
            battery_pct=self._last_telemetry.battery_pct,
            timestamp_ns=self._last_telemetry_time_ns if sensor_valid else None,
        )

        if self._ipc.send_telemetry(telemetry):

            self._telemetry_sequence = (
                self._telemetry_sequence + 1
            ) & 0xFFFFFFFF

            return True

        return False

    # ========================================================================
    # Telemetry freshness
    # ========================================================================

    def _telemetry_is_fresh(self) -> bool:

        if self._last_telemetry_time_ns is None:
            return False

        age_ns = (
            time.monotonic_ns()
            - self._last_telemetry_time_ns
        )

        maximum_age_ns = int(
            self._settings.telemetry_max_age_s
            * 1_000_000_000
        )

        return 0 <= age_ns <= maximum_age_ns

    def _check_telemetry_freshness(self) -> None:

        if self._state != BridgeState.NORMAL:
            return

        if not self._telemetry_is_fresh():

            self._enter_emergency(
                "Robot telemetry became stale"
            )


# ============================================================================
# Configuration loading
# ============================================================================

def _required_float(name: str) -> float:

    value = os.environ.get(name)

    if value is None:
        raise RuntimeError(
            f"Missing required environment variable: {name}"
        )

    try:
        result = float(value)

    except ValueError as exc:
        raise RuntimeError(
            f"Invalid numeric configuration: {name}"
        ) from exc

    if not math.isfinite(result):
        raise RuntimeError(
            f"{name} must be finite"
        )

    return result


def load_settings() -> BridgeSettings:

    ipc_config = BridgeConfig(
        receive_timeout_s=_required_float(
            "XGO_MQ_RECEIVE_TIMEOUT_S"
        ),
        supervisor_timeout_s=_required_float(
            "XGO_SUPERVISOR_TIMEOUT_S"
        ),
        startup_grace_s=_required_float(
            "XGO_STARTUP_GRACE_S"
        ),
        max_command_age_s=_required_float(
            "XGO_MAX_COMMAND_AGE_S"
        ),
        telemetry_send_timeout_s=_required_float(
            "XGO_MQ_SEND_TIMEOUT_S"
        ),
    )

    return BridgeSettings(
        ipc=ipc_config,

        telemetry_period_s=_required_float(
            "XGO_TELEMETRY_PERIOD_S"
        ),

        telemetry_max_age_s=_required_float(
            "XGO_TELEMETRY_MAX_AGE_S"
        ),

        recovery_max_pitch_deg=_required_float(
            "XGO_RECOVERY_MAX_PITCH_DEG"
        ),

        recovery_max_roll_deg=_required_float(
            "XGO_RECOVERY_MAX_ROLL_DEG"
        ),

        recovery_min_battery_pct=_required_float(
            "XGO_RECOVERY_MIN_BATTERY_PCT"
        ),
    )


# ============================================================================
# Process entry point
# ============================================================================

def main(hardware_factory: Callable[[], XgoHardware] | None = None) -> int:

    configure_logging()

    application = None

    def handle_shutdown_signal(signum, frame):

        if application is not None:
            application.request_shutdown()

    signal.signal(
        signal.SIGINT,
        handle_shutdown_signal,
    )

    signal.signal(
        signal.SIGTERM,
        handle_shutdown_signal,
    )

    try:

        settings = load_settings()

        if hardware_factory is None:
            baud = os.environ.get("XGO_UART_BAUD")
            hardware = XgolibRiderAdapter(
                port=os.environ.get("XGO_UART_PORT"),
                baud=int(baud) if baud is not None else None,
            )
        else:
            hardware = hardware_factory()
        supervisor_pid = os.environ.get("XGO_SUPERVISOR_PID")

        application = XgoBridgeApplication(
            settings=settings,
            hardware=hardware,
            telemetry_only=os.environ.get("XGO_TELEMETRY_ONLY") == "1",
            supervisor_pid=int(supervisor_pid) if supervisor_pid is not None else None,
        )

        application.start()
        application.run()

        return int(application.shutdown_failed)

    except Exception:

        logger.exception(
            "XGO bridge process failed"
        )

        return 1

    finally:

        if application is not None:
            application.close()


if __name__ == "__main__":
    sys.exit(main())
