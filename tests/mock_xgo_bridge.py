"""Run the production Python MQ bridge with simulated hardware."""

import json
import math
import os
from pathlib import Path

from bridge.xgo_bridge import main
from tests.mock_hardware import MockXgoHardware


class MockBridgeHardware(MockXgoHardware):
    def __init__(self):
        super().__init__()
        fault_path = os.environ.get("XGO_MOCK_TELEMETRY")
        event_path = os.environ.get("XGO_MOCK_EVENTS")
        self._fault_path = Path(fault_path) if fault_path else None
        self._event_path = Path(event_path) if event_path else None

    def _record(self, event, **fields):
        if self._event_path is not None:
            with self._event_path.open("a", encoding="utf-8") as stream:
                print(json.dumps({"event": event, **fields}), file=stream)

    def connect(self):
        super().connect()
        self._record("connected", pid=os.getpid())

    def move(self, vx, wz):
        self.vx = vx
        self.wz = wz
        self._record("move", vx=vx, wz=wz)

    def set_height(self, height_z):
        self.height_z = height_z
        self._record("height", height_z=height_z)

    def perform_action(self, action_id):
        self._record("action", action_id=action_id)

    def request_safe_response(self):
        super().request_safe_response()
        self._record("safe_response")

    def read_telemetry(self):
        if self._fault_path is not None:
            try:
                values = json.loads(self._fault_path.read_text(encoding="utf-8"))
            except FileNotFoundError:
                values = {}
            self.pitch_deg = float(values.get("pitch_deg", 0))
            self.roll_deg = float(values.get("roll_deg", 0))
            self.yaw_deg = float(values.get("yaw_deg", 0))
            self.battery_pct = float(values.get("battery_pct", 90))
            self.fail_telemetry = values.get("fail", False) or not all(
                math.isfinite(value) for value in (
                    self.pitch_deg, self.roll_deg, self.yaw_deg, self.battery_pct
                )
            ) or not 0 <= self.battery_pct <= 100
        return super().read_telemetry()

    def close(self):
        super().close()
        self._record("closed")


if __name__ == "__main__":
    raise SystemExit(main(MockBridgeHardware))
