import unittest
from unittest.mock import patch

from bridge.ipc_bridge import IpcBridge
from bridge.ipc_protocol import XgoCommandType, cmd_mq_priority, encode_cmd_message
from bridge.xgo_bridge import BridgeState
from tests.test_xgo_bridge import XgoBridgeTestCase, make_command, make_settings


class TestBridgeInterlock(XgoBridgeTestCase):
    def test_first_heartbeat_does_not_clear_startup_estop(self):
        self.start_application()
        self.ipc.commands.append(make_command(XgoCommandType.ESTOP))
        receive = self.ipc.receive_command

        def receive_with_heartbeat():
            command = receive()
            if command is None:
                self.ipc.heartbeat_received = True
            return command

        with patch.object(self.ipc, "receive_command", side_effect=receive_with_heartbeat):
            self.app._wait_for_supervisor()
        self.assertIs(self.app._state, BridgeState.EMERGENCY)

    def test_failed_telemetry_blocks_reset_until_a_new_sample(self):
        self.enter_normal_state()
        self.hardware.fail_telemetry = True
        self.app._last_telemetry_attempt_ns = 0
        self.app._poll_telemetry_if_due()
        self.app._attempt_recovery()
        self.assertIs(self.app._state, BridgeState.EMERGENCY)
        self.hardware.fail_telemetry = False
        self.app._refresh_telemetry()
        self.app._attempt_recovery()
        self.assertIs(self.app._state, BridgeState.NORMAL)

    def test_supervisor_expiry_during_receive_blocks_dispatch(self):
        self.enter_normal_state()
        movement = make_command(XgoCommandType.MOVE, vx=0.1)

        commands = iter((movement, make_command(XgoCommandType.SHUTDOWN)))

        def receive_then_expire():
            self.ipc.alive = False
            return next(commands)

        with patch.object(self.ipc, "receive_command", side_effect=receive_then_expire):
            self.app.run()
        self.assertFalse(any(call[0] == "move" for call in self.hardware.calls))

    def test_heartbeat_does_not_extend_motion_lease(self):
        self.enter_normal_state()
        self.app._dispatch_command(make_command(XgoCommandType.MOVE, vx=0.1))
        self.app._last_motion_time_ns = 1
        self.app._check_motion_expiry()
        self.assertIs(self.app._state, BridgeState.NORMAL)
        self.assertIsNone(self.app._last_motion_time_ns)
        self.assertEqual(self.hardware.calls.count(("request_safe_response",)), 1)


class TestPrioritySequence(unittest.TestCase):
    def test_priority_reordering_does_not_discard_older_heartbeat_or_admit_old_reset(self):
        ipc = IpcBridge(make_settings().ipc)
        heartbeat = make_command(XgoCommandType.HEARTBEAT, sequence=1)
        reset = make_command(XgoCommandType.RESET, sequence=2)
        stop = make_command(XgoCommandType.ESTOP, sequence=3)
        with patch.object(ipc, "_cmd_queue") as queue:
            queue.receive.side_effect = [
                (encode_cmd_message(command), cmd_mq_priority(command))
                for command in (stop, heartbeat, reset)
            ]
            self.assertEqual(ipc.receive_command().command_type, XgoCommandType.ESTOP)
            self.assertIsNone(ipc.receive_command())
            self.assertTrue(ipc.supervisor_heartbeat_received)
            self.assertIsNone(ipc.receive_command())
            self.assertTrue(ipc.emergency_active)
