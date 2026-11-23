
import sys
import types
import unittest

from unittest.mock import patch, MagicMock

from bridge.hardware.xgo_adapter import (
    XgolibRiderAdapter,
    RobotTelemetry,
)


# ============================================================================
# Fake vendor SDK
# ============================================================================

class FakeXGO:
    """
    Fake implementation of the vendor's xgolib.XGO class.

    Records SDK calls and allows tests to simulate hardware failures.
    """

    def __init__(self, robot_type):

        self.robot_type = robot_type

        self.calls = []

        self.firmware = "1.0.0"

        self.pitch = 2.5
        self.roll = -1.0
        self.yaw = 45.0
        self.battery = 90.0

        self.fail_firmware = False
        self.fail_telemetry = False
        self.fail_movement = False
        self.fail_close = False

        self.closed = False

    def read_firmware(self):

        if self.fail_firmware:
            raise RuntimeError("Simulated firmware failure")

        return self.firmware

    def rider_move_x(self, vx):

        if self.fail_movement:
            raise RuntimeError("Simulated movement failure")

        self.calls.append(("rider_move_x", vx))

    def rider_turn(self, wz):

        if self.fail_movement:
            raise RuntimeError("Simulated movement failure")

        self.calls.append(("rider_turn", wz))

    def rider_height(self, height_z):

        self.calls.append(("rider_height", height_z))

    def rider_action(self, action_id):

        self.calls.append(("rider_action", action_id))

    def rider_read_pitch(self):

        if self.fail_telemetry:
            raise RuntimeError("Simulated telemetry failure")

        return self.pitch

    def rider_read_roll(self):

        if self.fail_telemetry:
            raise RuntimeError("Simulated telemetry failure")

        return self.roll

    def rider_read_yaw(self):

        if self.fail_telemetry:
            raise RuntimeError("Simulated telemetry failure")

        return self.yaw

    def rider_read_battery(self):

        if self.fail_telemetry:
            raise RuntimeError("Simulated telemetry failure")

        return self.battery

    def close(self):

        self.calls.append(("close",))

        if self.fail_close:
            raise RuntimeError("Simulated close failure")

        self.closed = True


# ============================================================================
# Test fixture
# ============================================================================

class TestXgolibRiderAdapter(unittest.TestCase):
    """
    Unit tests for XgolibRiderAdapter.

    The vendor SDK is replaced with FakeXGO, so no physical
    hardware or installed xgolib package is required.
    """

    def setUp(self):

        self.fake_robot = None

        def fake_xgo_constructor(robot_type):

            self.fake_robot = FakeXGO(robot_type)

            return self.fake_robot

        fake_module = types.ModuleType("xgolib")

        fake_module.XGO = fake_xgo_constructor

        self.sdk_patcher = patch.dict(
            sys.modules,
            {"xgolib": fake_module},
        )

        self.sdk_patcher.start()

        self.adapter = XgolibRiderAdapter()

    def tearDown(self):

        self.sdk_patcher.stop()

    def connect_adapter(self):

        self.adapter.connect()

        self.assertIsNotNone(self.fake_robot)

        return self.fake_robot


# ============================================================================
# Connection tests
# ============================================================================

class TestConnection(TestXgolibRiderAdapter):
    """Test hardware initialization and connection lifecycle."""

    def test_successful_connection(self):

        self.adapter.connect()

        self.assertIsNotNone(self.fake_robot)

        self.assertEqual(
            self.fake_robot.robot_type,
            "rider",
        )

    def test_firmware_is_checked(self):

        robot = self.connect_adapter()

        self.assertEqual(robot.firmware, "1.0.0")

    def test_invalid_firmware_rejected(self):

        def fake_xgo_constructor(robot_type):

            self.fake_robot = FakeXGO(robot_type)

            self.fake_robot.firmware = None

            return self.fake_robot

        fake_module = types.ModuleType("xgolib")
        fake_module.XGO = fake_xgo_constructor

        with patch.dict(sys.modules, {"xgolib": fake_module}):

            with self.assertRaises(RuntimeError):
                self.adapter.connect()

        self.assertIsNone(self.adapter._robot)

    def test_firmware_exception_propagates(self):

        def fake_xgo_constructor(robot_type):

            self.fake_robot = FakeXGO(robot_type)

            self.fake_robot.fail_firmware = True

            return self.fake_robot

        fake_module = types.ModuleType("xgolib")
        fake_module.XGO = fake_xgo_constructor

        with patch.dict(sys.modules, {"xgolib": fake_module}):

            with self.assertRaises(RuntimeError):
                self.adapter.connect()

        self.assertIsNone(self.adapter._robot)

    def test_duplicate_connection_rejected(self):

        self.connect_adapter()

        with self.assertRaises(RuntimeError):
            self.adapter.connect()

    def test_failed_connection_closes_partial_initialization(self):

        def fake_xgo_constructor(robot_type):

            self.fake_robot = FakeXGO(robot_type)

            self.fake_robot.firmware = None

            return self.fake_robot

        fake_module = types.ModuleType("xgolib")
        fake_module.XGO = fake_xgo_constructor

        with patch.dict(sys.modules, {"xgolib": fake_module}):

            with self.assertRaises(RuntimeError):
                self.adapter.connect()

        self.assertTrue(
            self.fake_robot.closed
        )


# ============================================================================
# Movement tests
# ============================================================================

class TestMovement(TestXgolibRiderAdapter):
    """Test movement command translation and validation."""

    def test_move_calls_correct_sdk_methods(self):

        robot = self.connect_adapter()

        self.adapter.move(0.5, 90.0)

        self.assertEqual(
            robot.calls,
            [
                ("rider_move_x", 0.5),
                ("rider_turn", 90.0),
            ],
        )

    def test_move_converts_values_to_float(self):

        robot = self.connect_adapter()

        self.adapter.move(1, 90)

        self.assertEqual(
            robot.calls,
            [
                ("rider_move_x", 1.0),
                ("rider_turn", 90.0),
            ],
        )

    def test_move_accepts_boundary_values(self):

        self.connect_adapter()

        self.adapter.move(-1.5, -360.0)
        self.adapter.move(1.5, 360.0)

    def test_move_rejects_velocity_outside_limits(self):

        self.connect_adapter()

        invalid_values = (
            (-1.6, 0.0),
            (1.6, 0.0),
            (0.0, -360.1),
            (0.0, 360.1),
        )

        for vx, wz in invalid_values:

            with self.subTest(vx=vx, wz=wz):

                with self.assertRaises(ValueError):
                    self.adapter.move(vx, wz)

    def test_move_rejects_non_finite_values(self):

        self.connect_adapter()

        invalid_values = (
            (float("nan"), 0.0),
            (float("inf"), 0.0),
            (0.0, float("-inf")),
        )

        for vx, wz in invalid_values:

            with self.subTest(vx=vx, wz=wz):

                with self.assertRaises(ValueError):
                    self.adapter.move(vx, wz)

    def test_move_rejects_disconnected_adapter(self):

        with self.assertRaises(RuntimeError):
            self.adapter.move(0.5, 0.0)

    def test_sdk_movement_failure_propagates(self):

        robot = self.connect_adapter()

        robot.fail_movement = True

        with self.assertRaises(RuntimeError):
            self.adapter.move(0.5, 0.0)


# ============================================================================
# Height tests
# ============================================================================

class TestHeight(TestXgolibRiderAdapter):
    """Test height command translation and validation."""

    def test_height_calls_correct_sdk_method(self):

        robot = self.connect_adapter()

        self.adapter.set_height(90)

        self.assertEqual(
            robot.calls,
            [
                ("rider_height", 90.0),
            ],
        )

    def test_height_accepts_boundary_values(self):

        self.connect_adapter()

        self.adapter.set_height(60)
        self.adapter.set_height(120)

    def test_height_rejects_values_outside_limits(self):

        self.connect_adapter()

        for height in (59.9, 120.1):

            with self.subTest(height=height):

                with self.assertRaises(ValueError):
                    self.adapter.set_height(height)

    def test_height_rejects_non_finite_values(self):

        self.connect_adapter()

        for height in (
            float("nan"),
            float("inf"),
            float("-inf"),
        ):

            with self.subTest(height=height):

                with self.assertRaises(ValueError):
                    self.adapter.set_height(height)

    def test_height_rejects_disconnected_adapter(self):

        with self.assertRaises(RuntimeError):
            self.adapter.set_height(90)


# ============================================================================
# Action tests
# ============================================================================

class TestActions(TestXgolibRiderAdapter):
    """Test action command translation and validation."""

    def test_action_calls_correct_sdk_method(self):

        robot = self.connect_adapter()

        self.adapter.perform_action(5)

        self.assertEqual(
            robot.calls,
            [
                ("rider_action", 5),
            ],
        )

    def test_action_accepts_boundary_values(self):

        self.connect_adapter()

        self.adapter.perform_action(1)
        self.adapter.perform_action(255)

    def test_action_rejects_invalid_ids(self):

        self.connect_adapter()

        for action_id in (0, 256, -1):

            with self.subTest(action_id=action_id):

                with self.assertRaises(ValueError):
                    self.adapter.perform_action(action_id)

    def test_action_rejects_disconnected_adapter(self):

        with self.assertRaises(RuntimeError):
            self.adapter.perform_action(1)


# ============================================================================
# Telemetry tests
# ============================================================================

class TestTelemetry(TestXgolibRiderAdapter):
    """Test sensor acquisition and telemetry validation."""

    def test_read_telemetry_returns_robot_telemetry(self):

        self.connect_adapter()

        telemetry = self.adapter.read_telemetry()

        self.assertIsInstance(
            telemetry,
            RobotTelemetry,
        )

    def test_read_telemetry_returns_correct_values(self):

        robot = self.connect_adapter()

        telemetry = self.adapter.read_telemetry()

        self.assertEqual(telemetry.pitch_deg, robot.pitch)
        self.assertEqual(telemetry.roll_deg, robot.roll)
        self.assertEqual(telemetry.yaw_deg, robot.yaw)
        self.assertEqual(telemetry.battery_pct, robot.battery)

    def test_read_telemetry_converts_values_to_float(self):

        robot = self.connect_adapter()

        robot.pitch = 2
        robot.roll = -1
        robot.yaw = 45
        robot.battery = 90

        telemetry = self.adapter.read_telemetry()

        self.assertIsInstance(telemetry.pitch_deg, float)
        self.assertIsInstance(telemetry.roll_deg, float)
        self.assertIsInstance(telemetry.yaw_deg, float)
        self.assertIsInstance(telemetry.battery_pct, float)

    def test_invalid_sensor_values_rejected(self):

        robot = self.connect_adapter()

        invalid_values = (
            ("pitch", float("nan")),
            ("roll", float("inf")),
            ("yaw", float("-inf")),
        )

        for field, value in invalid_values:

            with self.subTest(field=field):

                setattr(robot, field, value)

                with self.assertRaises(RuntimeError):
                    self.adapter.read_telemetry()

                setattr(robot, field, 0.0)

    def test_invalid_battery_values_rejected(self):

        robot = self.connect_adapter()

        for battery in (-1.0, 100.1):

            with self.subTest(battery=battery):

                robot.battery = battery

                with self.assertRaises(RuntimeError):
                    self.adapter.read_telemetry()

    def test_telemetry_sdk_failure_propagates(self):

        robot = self.connect_adapter()

        robot.fail_telemetry = True

        with self.assertRaises(RuntimeError):
            self.adapter.read_telemetry()

    def test_telemetry_rejects_disconnected_adapter(self):

        with self.assertRaises(RuntimeError):
            self.adapter.read_telemetry()


# ============================================================================
# Safe response tests
# ============================================================================

class TestSafeResponse(TestXgolibRiderAdapter):
    """Test the adapter's safe-response behaviour."""

    def test_safe_response_requests_zero_velocity(self):

        robot = self.connect_adapter()

        self.adapter.request_safe_response()

        self.assertEqual(
            robot.calls,
            [
                ("rider_move_x", 0.0),
                ("rider_turn", 0.0),
            ],
        )

    def test_safe_response_rejects_disconnected_adapter(self):

        with self.assertRaises(RuntimeError):
            self.adapter.request_safe_response()


# ============================================================================
# Disconnection tests
# ============================================================================

class TestDisconnection(TestXgolibRiderAdapter):
    """Test hardware resource cleanup."""

    def test_close_calls_sdk_close(self):

        robot = self.connect_adapter()

        self.adapter.close()

        self.assertTrue(robot.closed)

    def test_close_clears_internal_robot_reference(self):

        self.connect_adapter()

        self.adapter.close()

        self.assertIsNone(self.adapter._robot)

    def test_close_is_idempotent(self):

        robot = self.connect_adapter()

        self.adapter.close()
        self.adapter.close()

        self.assertEqual(
            robot.calls.count(("close",)),
            1,
        )

    def test_close_without_connection(self):

        self.adapter.close()

        self.assertIsNone(self.adapter._robot)

    def test_close_clears_reference_even_if_sdk_close_fails(self):

        robot = self.connect_adapter()

        robot.fail_close = True

        with self.assertRaises(RuntimeError):
            self.adapter.close()

        self.assertIsNone(self.adapter._robot)


if __name__ == "__main__":
    unittest.main()
