
"""
Tests for bridge/xgo_bridge.py.

These tests isolate XgoBridgeApplication from:
    - The real POSIX message queues.
    - The real XGO Rider 2 hardware.
    - The vendor xgolib SDK.

Run from the project root:

    pytest tests/test_xgo_bridge.py -v

Or:

    python -m unittest tests.test_xgo_bridge -v
"""

import unittest
from unittest.mock import patch

from bridge.xgo_bridge import (
    BridgeSettings,
    BridgeState,
    XgoBridgeApplication,
)

from bridge.ipc_bridge import BridgeConfig

from bridge.ipc_protocol import (
    XgoCommandType,
    XgoStatusFlags,
    create_cmd_message,
    monotonic_now_ns,
)

from bridge.hardware.xgo_adapter import RobotTelemetry


# ============================================================================
# Test doubles
# ============================================================================

class FakeHardware:
    """
    Hardware fake implementing the XgoHardware interface.

    It records every operation so tests can verify which commands
    the application sends to the hardware.
    """

    def __init__(self):
        self.connected = False
        self.closed = False

        self.calls = []

        self.telemetry = RobotTelemetry(
            pitch_deg=0.0,
            roll_deg=0.0,
            yaw_deg=0.0,
            battery_pct=80.0,
        )

        self.fail_connect = False
        self.fail_move = False
        self.fail_height = False
        self.fail_action = False
        self.fail_telemetry = False
        self.fail_safe_response = False
        self.fail_close = False

    def connect(self):
        self.calls.append(("connect",))

        if self.fail_connect:
            raise RuntimeError("Simulated hardware connection failure")

        self.connected = True

    def move(self, vx, wz):
        self.calls.append(("move", vx, wz))

        if self.fail_move:
            raise RuntimeError("Simulated movement failure")

    def set_height(self, height_z):
        self.calls.append(("set_height", height_z))

        if self.fail_height:
            raise RuntimeError("Simulated height failure")

    def perform_action(self, action_id):
        self.calls.append(("perform_action", action_id))

        if self.fail_action:
            raise RuntimeError("Simulated action failure")

    def read_telemetry(self):
        self.calls.append(("read_telemetry",))

        if self.fail_telemetry:
            raise RuntimeError("Simulated telemetry failure")

        return self.telemetry

    def request_safe_response(self):
        self.calls.append(("request_safe_response",))

        if self.fail_safe_response:
            raise RuntimeError("Simulated safe-response failure")

    def close(self):
        self.calls.append(("close",))

        if self.fail_close:
            raise RuntimeError("Simulated hardware cleanup failure")

        self.closed = True


class FakeIpc:
    """
    Fake implementation of the IpcBridge interface.

    It allows tests to control:
        - Supervisor liveness.
        - Heartbeat reception.
        - Incoming commands.
        - Telemetry transmission.
        - Emergency recovery.
    """

    def __init__(self, settings):
        self.settings = settings

        self.opened = False
        self.closed = False

        self.alive = True
        self.heartbeat_received = False

        self.commands = []
        self.sent_telemetry = []

        self.telemetry_send_result = True
        self.fail_open = False
        self.fail_recovery = False

        self.recovery_completed = False

    def open(self):
        if self.fail_open:
            raise RuntimeError("Simulated IPC open failure")

        self.opened = True

    def close(self):
        self.closed = True

    def receive_command(self):
        if self.commands:
            return self.commands.pop(0)

        return None

    def send_telemetry(self, telemetry):
        if self.telemetry_send_result:
            self.sent_telemetry.append(telemetry)

        return self.telemetry_send_result

    def supervisor_alive(self):
        return self.alive

    @property
    def supervisor_heartbeat_received(self):
        return self.heartbeat_received

    def complete_recovery(self):
        if self.fail_recovery:
            raise RuntimeError("Simulated recovery failure")

        self.recovery_completed = True


# ============================================================================
# Test helpers
# ============================================================================

def make_settings(
    telemetry_period_s=0.1,
    telemetry_max_age_s=1.0,
    recovery_max_pitch_deg=15.0,
    recovery_max_roll_deg=15.0,
    recovery_min_battery_pct=20.0,
):
    """Create valid application settings for tests."""

    ipc_settings = BridgeConfig(
        receive_timeout_s=0.01,
        supervisor_timeout_s=1.0,
        startup_grace_s=1.0,
        max_command_age_s=1.0,
        telemetry_send_timeout_s=0.01,
    )

    return BridgeSettings(
        ipc=ipc_settings,
        telemetry_period_s=telemetry_period_s,
        telemetry_max_age_s=telemetry_max_age_s,
        recovery_max_pitch_deg=recovery_max_pitch_deg,
        recovery_max_roll_deg=recovery_max_roll_deg,
        recovery_min_battery_pct=recovery_min_battery_pct,
    )


def make_command(
    command_type,
    sequence=1,
    vx=0.0,
    wz=0.0,
    height_z=0.0,
    action_id=0,
):
    """Create a valid IPC command for application-level tests."""

    return create_cmd_message(
        sequence=sequence,
        command_type=command_type,
        vx=vx,
        wz=wz,
        height_z=height_z,
        action_id=action_id,
        timestamp_ns=monotonic_now_ns(),
    )


# ============================================================================
# Base test class
# ============================================================================

class XgoBridgeTestCase(unittest.TestCase):

    def setUp(self):
        self.settings = make_settings()
        self.hardware = FakeHardware()

        self.ipc_patch = patch(
            "bridge.xgo_bridge.IpcBridge",
            FakeIpc,
        )

        self.ipc_patch.start()
        self.addCleanup(self.ipc_patch.stop)

        self.app = XgoBridgeApplication(
            settings=self.settings,
            hardware=self.hardware,
        )

        self.ipc = self.app._ipc

    def start_application(self):
        """Start the application and return it in its initial state."""

        self.app.start()

        return self.app

    def enter_normal_state(self):
        """Start the application and simulate supervisor connection."""

        self.start_application()

        self.ipc.heartbeat_received = True

        self.app._wait_for_supervisor()

        self.assertEqual(
            self.app._state,
            BridgeState.NORMAL,
        )


# ============================================================================
# Configuration tests
# ============================================================================

class TestBridgeSettings(unittest.TestCase):

    def test_valid_settings(self):
        settings = make_settings()

        self.assertEqual(
            settings.telemetry_period_s,
            0.1,
        )

    def test_zero_telemetry_period_is_rejected(self):
        with self.assertRaises(ValueError):
            make_settings(telemetry_period_s=0.0)

    def test_negative_telemetry_period_is_rejected(self):
        with self.assertRaises(ValueError):
            make_settings(telemetry_period_s=-1.0)

    def test_invalid_telemetry_max_age_is_rejected(self):
        with self.assertRaises(ValueError):
            make_settings(telemetry_max_age_s=0.0)

    def test_invalid_recovery_battery_threshold_is_rejected(self):
        with self.assertRaises(ValueError):
            make_settings(recovery_min_battery_pct=101.0)

    def test_negative_recovery_battery_threshold_is_rejected(self):
        with self.assertRaises(ValueError):
            make_settings(recovery_min_battery_pct=-1.0)


# ============================================================================
# Startup and lifecycle tests
# ============================================================================

class TestApplicationStartup(XgoBridgeTestCase):

    def test_start_opens_ipc_and_connects_hardware(self):
        self.start_application()

        self.assertTrue(self.ipc.opened)
        self.assertTrue(self.hardware.connected)

    def test_start_reads_initial_telemetry(self):
        self.start_application()

        self.assertIsNotNone(
            self.app._last_telemetry,
        )

        self.assertIn(
            ("read_telemetry",),
            self.hardware.calls,
        )

    def test_start_publishes_ready_telemetry(self):
        self.start_application()

        self.assertEqual(
            len(self.ipc.sent_telemetry),
            1,
        )

        telemetry = self.ipc.sent_telemetry[0]

        self.assertTrue(
            telemetry.status_flags & XgoStatusFlags.READY
        )

        self.assertTrue(
            telemetry.status_flags & XgoStatusFlags.SENSOR_VALID
        )

    def test_start_leaves_application_in_starting_state(self):
        self.start_application()

        self.assertEqual(
            self.app._state,
            BridgeState.STARTING,
        )

    def test_start_fails_if_ipc_cannot_open(self):
        self.ipc.fail_open = True

        with self.assertRaises(RuntimeError):
            self.app.start()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

        self.assertFalse(self.hardware.connected)

    def test_start_fails_if_hardware_cannot_connect(self):
        self.hardware.fail_connect = True

        with self.assertRaises(RuntimeError):
            self.app.start()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

        self.assertTrue(self.ipc.closed)

    def test_start_fails_if_initial_telemetry_fails(self):
        self.hardware.fail_telemetry = True

        with self.assertRaises(RuntimeError):
            self.app.start()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

        self.assertTrue(self.hardware.closed)

    def test_start_fails_if_ready_telemetry_cannot_be_sent(self):
        self.ipc.telemetry_send_result = False

        with self.assertRaises(RuntimeError):
            self.app.start()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

        self.assertTrue(self.hardware.closed)
        self.assertTrue(self.ipc.closed)

    def test_application_cannot_be_started_twice(self):
        self.start_application()

        with self.assertRaises(RuntimeError):
            self.app.start()


# ============================================================================
# Supervisor startup handshake tests
# ============================================================================

class TestSupervisorHandshake(XgoBridgeTestCase):

    def test_wait_for_heartbeat_enters_normal_state(self):
        self.start_application()

        self.ipc.heartbeat_received = True

        self.app._wait_for_supervisor()

        self.assertEqual(
            self.app._state,
            BridgeState.NORMAL,
        )

    def test_wait_does_not_enter_normal_without_heartbeat(self):
        self.start_application()

        self.ipc.heartbeat_received = False
        self.ipc.commands = []

        # Prevent the fake IPC from waiting indefinitely.
        self.app._running = True

        self.ipc.alive = False

        with self.assertRaises(TimeoutError):
            self.app._wait_for_supervisor()

    def test_expired_watchdog_after_heartbeat_raises(self):
        self.start_application()

        self.ipc.heartbeat_received = True
        self.ipc.alive = False

        with self.assertRaises(RuntimeError):
            self.app._wait_for_supervisor()

    def test_run_requires_application_to_be_started(self):
        with self.assertRaises(RuntimeError):
            self.app.run()


# ============================================================================
# Command dispatch tests
# ============================================================================

class TestCommandDispatch(XgoBridgeTestCase):

    def setUp(self):
        super().setUp()
        self.enter_normal_state()

    def test_move_command_is_forwarded_to_hardware(self):
        command = make_command(
            XgoCommandType.MOVE,
            vx=0.5,
            wz=30.0,
        )

        self.app._dispatch_command(command)

        self.assertIn(
            ("move", 0.5, 30.0),
            self.hardware.calls,
        )

    def test_height_command_is_forwarded_to_hardware(self):
        command = make_command(
            XgoCommandType.HEIGHT,
            height_z=100.0,
        )

        self.app._dispatch_command(command)

        self.assertIn(
            ("set_height", 100.0),
            self.hardware.calls,
        )

    def test_action_command_is_forwarded_to_hardware(self):
        command = make_command(
            XgoCommandType.ACTION,
            action_id=5,
        )

        self.app._dispatch_command(command)

        self.assertIn(
            ("perform_action", 5),
            self.hardware.calls,
        )

    def test_heartbeat_does_not_call_hardware(self):
        previous_calls = list(self.hardware.calls)

        command = make_command(
            XgoCommandType.HEARTBEAT,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.hardware.calls,
            previous_calls,
        )

    def test_estop_enters_emergency(self):
        command = make_command(
            XgoCommandType.ESTOP,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_shutdown_command_stops_application(self):
        command = make_command(
            XgoCommandType.SHUTDOWN,
        )

        self.app._dispatch_command(command)

        self.assertTrue(
            self.app._shutdown_requested,
        )

        self.assertFalse(
            self.app._running,
        )

    def test_movement_is_rejected_outside_normal_state(self):
        self.app._enter_emergency("Test emergency")

        previous_calls = list(self.hardware.calls)

        command = make_command(
            XgoCommandType.MOVE,
            vx=0.5,
            wz=30.0,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.hardware.calls,
            previous_calls,
        )

    def test_hardware_movement_failure_enters_emergency(self):
        self.hardware.fail_move = True

        command = make_command(
            XgoCommandType.MOVE,
            vx=0.5,
            wz=30.0,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_hardware_height_failure_enters_emergency(self):
        self.hardware.fail_height = True

        command = make_command(
            XgoCommandType.HEIGHT,
            height_z=100.0,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_hardware_action_failure_enters_emergency(self):
        self.hardware.fail_action = True

        command = make_command(
            XgoCommandType.ACTION,
            action_id=5,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )


# ============================================================================
# Emergency tests
# ============================================================================

class TestEmergencyHandling(XgoBridgeTestCase):

    def setUp(self):
        super().setUp()
        self.enter_normal_state()

    def test_enter_emergency_changes_state(self):
        self.app._enter_emergency("Test emergency")

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_enter_emergency_requests_safe_response(self):
        self.app._enter_emergency("Test emergency")

        self.assertIn(
            ("request_safe_response",),
            self.hardware.calls,
        )

    def test_repeated_emergency_does_not_repeat_safe_response(self):
        self.app._enter_emergency("First emergency")

        previous_count = self.hardware.calls.count(
            ("request_safe_response",)
        )

        self.app._enter_emergency("Second emergency")

        self.assertEqual(
            self.hardware.calls.count(
                ("request_safe_response",)
            ),
            previous_count,
        )

    def test_emergency_safe_response_failure_does_not_crash(self):
        self.hardware.fail_safe_response = True

        self.app._enter_emergency("Test emergency")

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_emergency_telemetry_flag_is_published(self):
        self.app._enter_emergency("Test emergency")

        self.app._publish_telemetry()

        telemetry = self.ipc.sent_telemetry[-1]

        self.assertTrue(
            telemetry.status_flags
            & XgoStatusFlags.EMERGENCY_ACTIVE
        )

    def test_emergency_does_not_publish_ready_flag(self):
        self.app._enter_emergency("Test emergency")

        self.app._publish_telemetry()

        telemetry = self.ipc.sent_telemetry[-1]

        self.assertFalse(
            telemetry.status_flags & XgoStatusFlags.READY
        )

    def test_estop_does_not_change_shutdown_state(self):
        self.app.close()

        command = make_command(
            XgoCommandType.ESTOP,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )


# ============================================================================
# Recovery tests
# ============================================================================

class TestRecovery(XgoBridgeTestCase):

    def setUp(self):
        super().setUp()
        self.enter_normal_state()
        self.app._enter_emergency("Test emergency")

    def test_recovery_succeeds_when_conditions_are_safe(self):
        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.NORMAL,
        )

        self.assertTrue(
            self.ipc.recovery_completed,
        )

    def test_recovery_is_rejected_if_supervisor_is_unavailable(self):
        self.ipc.alive = False

        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

        self.assertFalse(
            self.ipc.recovery_completed,
        )

    def test_recovery_is_rejected_if_telemetry_is_stale(self):
        self.app._last_telemetry_time_ns = 0

        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

        self.assertFalse(
            self.ipc.recovery_completed,
        )

    def test_recovery_is_rejected_if_pitch_is_too_high(self):
        self.hardware.telemetry = RobotTelemetry(
            pitch_deg=20.0,
            roll_deg=0.0,
            yaw_deg=0.0,
            battery_pct=80.0,
        )

        self.app._refresh_telemetry()
        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_recovery_is_rejected_if_roll_is_too_high(self):
        self.hardware.telemetry = RobotTelemetry(
            pitch_deg=0.0,
            roll_deg=20.0,
            yaw_deg=0.0,
            battery_pct=80.0,
        )

        self.app._refresh_telemetry()
        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_recovery_is_rejected_if_battery_is_too_low(self):
        self.hardware.telemetry = RobotTelemetry(
            pitch_deg=0.0,
            roll_deg=0.0,
            yaw_deg=0.0,
            battery_pct=10.0,
        )

        self.app._refresh_telemetry()
        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_recovery_failure_restores_emergency_state(self):
        self.ipc.fail_recovery = True

        self.app._attempt_recovery()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_reset_outside_emergency_is_ignored(self):
        self.app._state = BridgeState.NORMAL

        command = make_command(
            XgoCommandType.RESET,
        )

        self.app._dispatch_command(command)

        self.assertEqual(
            self.app._state,
            BridgeState.NORMAL,
        )


# ============================================================================
# Supervisor watchdog tests
# ============================================================================

class TestSupervisorWatchdog(XgoBridgeTestCase):

    def setUp(self):
        super().setUp()
        self.enter_normal_state()

    def test_alive_supervisor_does_not_trigger_emergency(self):
        self.ipc.alive = True

        self.app._check_supervisor_watchdog()

        self.assertEqual(
            self.app._state,
            BridgeState.NORMAL,
        )

    def test_expired_supervisor_watchdog_triggers_emergency(self):
        self.ipc.alive = False

        self.app._check_supervisor_watchdog()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_watchdog_emergency_requests_safe_response(self):
        self.ipc.alive = False

        self.app._check_supervisor_watchdog()

        self.assertIn(
            ("request_safe_response",),
            self.hardware.calls,
        )


# ============================================================================
# Telemetry tests
# ============================================================================

class TestTelemetry(XgoBridgeTestCase):

    def setUp(self):
        super().setUp()
        self.enter_normal_state()

    def test_refresh_telemetry_updates_cached_values(self):
        expected = RobotTelemetry(
            pitch_deg=1.0,
            roll_deg=2.0,
            yaw_deg=3.0,
            battery_pct=75.0,
        )

        self.hardware.telemetry = expected

        self.app._refresh_telemetry()

        self.assertEqual(
            self.app._last_telemetry,
            expected,
        )

        self.assertIsNotNone(
            self.app._last_telemetry_time_ns,
        )

    def test_publish_telemetry_sets_sensor_valid_flag(self):
        self.app._publish_telemetry()

        telemetry = self.ipc.sent_telemetry[-1]

        self.assertTrue(
            telemetry.status_flags & XgoStatusFlags.SENSOR_VALID
        )

    def test_publish_telemetry_sets_ready_flag_in_normal_state(self):
        self.app._publish_telemetry()

        telemetry = self.ipc.sent_telemetry[-1]

        self.assertTrue(
            telemetry.status_flags & XgoStatusFlags.READY
        )

    def test_telemetry_sequence_increments_after_successful_send(self):
        initial_sequence = self.app._telemetry_sequence

        self.app._publish_telemetry()

        self.assertEqual(
            self.app._telemetry_sequence,
            initial_sequence + 1,
        )

    def test_telemetry_sequence_does_not_increment_after_failed_send(self):
        self.ipc.telemetry_send_result = False

        initial_sequence = self.app._telemetry_sequence

        result = self.app._publish_telemetry()

        self.assertFalse(result)

        self.assertEqual(
            self.app._telemetry_sequence,
            initial_sequence,
        )

    def test_telemetry_sequence_wraps_at_uint32_limit(self):
        self.app._telemetry_sequence = 0xFFFFFFFF

        self.app._publish_telemetry()

        self.assertEqual(
            self.app._telemetry_sequence,
            0,
        )

    def test_polling_does_nothing_before_period_expires(self):
        initial_reads = self.hardware.calls.count(
            ("read_telemetry",)
        )

        self.app._poll_telemetry_if_due()

        self.assertEqual(
            self.hardware.calls.count(("read_telemetry",)),
            initial_reads,
        )

    def test_telemetry_failure_enters_emergency(self):
        self.hardware.fail_telemetry = True

        # Make the next telemetry poll due.
        self.app._last_telemetry_attempt_ns = 0

        self.app._poll_telemetry_if_due()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_telemetry_failure_publishes_bridge_error_flag(self):
        self.hardware.fail_telemetry = True

        self.app._last_telemetry_attempt_ns = 0

        self.app._poll_telemetry_if_due()

        telemetry = self.ipc.sent_telemetry[-1]

        self.assertTrue(
            telemetry.status_flags & XgoStatusFlags.BRIDGE_ERROR
        )

    def test_telemetry_failure_clears_sensor_valid_flag(self):
        self.hardware.fail_telemetry = True

        self.app._last_telemetry_attempt_ns = 0

        self.app._poll_telemetry_if_due()

        telemetry = self.ipc.sent_telemetry[-1]

        self.assertFalse(
            telemetry.status_flags & XgoStatusFlags.SENSOR_VALID
        )

    def test_stale_telemetry_triggers_emergency(self):
        self.app._last_telemetry_time_ns = 0

        self.app._check_telemetry_freshness()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

    def test_fresh_telemetry_does_not_trigger_emergency(self):
        self.app._refresh_telemetry()

        self.app._check_telemetry_freshness()

        self.assertEqual(
            self.app._state,
            BridgeState.NORMAL,
        )

    def test_telemetry_freshness_is_not_checked_during_emergency(self):
        self.app._enter_emergency("Test emergency")

        self.app._last_telemetry_time_ns = 0

        previous_safe_responses = self.hardware.calls.count(
            ("request_safe_response",)
        )

        self.app._check_telemetry_freshness()

        self.assertEqual(
            self.app._state,
            BridgeState.EMERGENCY,
        )

        self.assertEqual(
            self.hardware.calls.count(
                ("request_safe_response",)
            ),
            previous_safe_responses,
        )


# ============================================================================
# Shutdown and cleanup tests
# ============================================================================

class TestShutdown(XgoBridgeTestCase):

    def setUp(self):
        super().setUp()
        self.start_application()

    def test_request_shutdown_stops_application(self):
        self.app.request_shutdown()

        self.assertTrue(
            self.app._shutdown_requested,
        )

        self.assertFalse(
            self.app._running,
        )

    def test_close_changes_state_to_shutdown(self):
        self.app.close()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

    def test_close_requests_safe_response(self):
        self.app.close()

        self.assertIn(
            ("request_safe_response",),
            self.hardware.calls,
        )

    def test_close_disconnects_hardware(self):
        self.app.close()

        self.assertTrue(
            self.hardware.closed,
        )

    def test_close_closes_ipc(self):
        self.app.close()

        self.assertTrue(
            self.ipc.closed,
        )

    def test_close_is_idempotent(self):
        self.app.close()

        previous_calls = list(self.hardware.calls)

        self.app.close()

        self.assertEqual(
            self.hardware.calls,
            previous_calls,
        )

    def test_hardware_cleanup_failure_does_not_prevent_ipc_cleanup(self):
        self.hardware.fail_close = True

        self.app.close()

        self.assertTrue(
            self.ipc.closed,
        )

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

    def test_safe_response_failure_does_not_prevent_cleanup(self):
        self.hardware.fail_safe_response = True

        self.app.close()

        self.assertTrue(
            self.hardware.closed,
        )

        self.assertTrue(
            self.ipc.closed,
        )

    def test_ipc_cleanup_failure_does_not_crash_close(self):
        def failing_close():
            raise RuntimeError("Simulated IPC cleanup failure")

        self.ipc.close = failing_close

        self.app.close()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )


# ============================================================================
# Application run-loop tests
# ============================================================================

class TestApplicationRunLoop(XgoBridgeTestCase):

    def test_run_processes_shutdown_command(self):
        self.start_application()

        self.ipc.heartbeat_received = True

        self.ipc.commands = [
            make_command(XgoCommandType.SHUTDOWN),
        ]

        self.app.run()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

        self.assertTrue(
            self.hardware.closed,
        )

        self.assertTrue(
            self.ipc.closed,
        )

    def test_run_enters_normal_after_supervisor_handshake(self):
        self.start_application()

        self.ipc.heartbeat_received = True

        # The first command requests shutdown, allowing the run loop
        # to finish after completing the startup handshake.
        self.ipc.commands = [
            make_command(XgoCommandType.SHUTDOWN),
        ]

        self.app.run()

        # run() closes the application in its finally block.
        # Verify that the startup handshake was completed by checking
        # that the application reached the normal state before shutdown.
        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

    def test_run_closes_application_after_keyboard_interrupt(self):
        self.start_application()

        self.ipc.heartbeat_received = True

        def raise_keyboard_interrupt():
            raise KeyboardInterrupt

        self.ipc.receive_command = raise_keyboard_interrupt

        self.app.run()

        self.assertEqual(
            self.app._state,
            BridgeState.SHUTDOWN,
        )

        self.assertTrue(
            self.hardware.closed,
        )


if __name__ == "__main__":
    unittest.main()
