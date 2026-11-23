"""In-process hardware substitute shared by bridge tests and the mock child."""

from bridge.hardware.xgo_adapter import RobotTelemetry


class MockXgoHardware:
    def __init__(self):
        self.connected = False
        self.vx = 0.0
        self.wz = 0.0
        self.height_z = 90.0
        self.battery_pct = 90.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.yaw_deg = 0.0
        self.emergency_requested = False
        self.commands = []
        self.fail_connection = False
        self.fail_telemetry = False

    def connect(self):
        if self.fail_connection:
            raise RuntimeError("Simulated connection failure")
        self.connected = True

    def move(self, vx, wz):
        self.commands.append(("move", vx, wz))
        self.vx = vx
        self.wz = wz

    def set_height(self, height_z):
        self.commands.append(("height", height_z))
        self.height_z = height_z

    def perform_action(self, action_id):
        self.commands.append(("action", action_id))

    def read_telemetry(self):
        if self.fail_telemetry:
            raise RuntimeError("Simulated telemetry failure")
        return RobotTelemetry(
            pitch_deg=self.pitch_deg,
            roll_deg=self.roll_deg,
            yaw_deg=self.yaw_deg,
            battery_pct=self.battery_pct,
        )

    def request_safe_response(self):
        self.emergency_requested = True
        self.vx = 0.0
        self.wz = 0.0

    def close(self):
        self.connected = False
