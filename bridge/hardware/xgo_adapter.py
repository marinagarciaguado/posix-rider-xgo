
"""
hardware/xgo_adapter.py

Hardware abstraction for the XGO Rider 2.

This module exclusively owns the xgolib instance and UART connection.
"""

from __future__ import annotations

import logging
import math

from dataclasses import dataclass
from typing import Protocol


logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class RobotTelemetry:
    pitch_deg: float
    roll_deg: float
    yaw_deg: float
    battery_pct: float


class XgoHardware(Protocol):

    def connect(self) -> None:
        ...

    def move(self, vx: float, wz: float) -> None:
        ...

    def set_height(self, height_z: float) -> None:
        ...

    def perform_action(self, action_id: int) -> None:
        ...

    def read_telemetry(self) -> RobotTelemetry:
        ...

    def request_safe_response(self) -> None:
        ...

    def close(self) -> None:
        ...


class XgolibRiderAdapter:

    MIN_VX = -1.5
    MAX_VX = 1.5

    MIN_WZ = -360.0
    MAX_WZ = 360.0

    MIN_HEIGHT = 60.0
    MAX_HEIGHT = 120.0

    MIN_ACTION_ID = 1
    MAX_ACTION_ID = 255

    def __init__(self, *, port: str | None = None, baud: int | None = None) -> None:
        self._robot = None
        self._connection_options = {}
        if port is not None:
            self._connection_options["port"] = port
        if baud is not None:
            if baud < 0:
                raise ValueError("UART baud must be nonnegative")
            self._connection_options["baud"] = baud

    def connect(self) -> None:

        if self._robot is not None:
            raise RuntimeError("Hardware is already connected")

        from xgolib import XGO

        logger.info("Initializing XGO Rider hardware")

        robot = XGO("rider", **self._connection_options)

        try:
            firmware = robot.read_firmware()

            if not firmware:
                raise RuntimeError(
                    "Unable to identify robot firmware"
                )

        except Exception:
            self._robot = robot
            self.close()
            raise

        self._robot = robot

        logger.info(
            "Connected to XGO Rider firmware %s",
            firmware,
        )

    def _require_connection(self):

        if self._robot is None:
            raise RuntimeError("XGO hardware is not connected")

        return self._robot

    @staticmethod
    def _require_range(
        name: str,
        value: float,
        minimum: float,
        maximum: float,
    ) -> None:

        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")

        if not minimum <= value <= maximum:
            raise ValueError(
                f"{name} must be between {minimum} and {maximum}"
            )

    def move(self, vx: float, wz: float) -> None:

        self._require_range(
            "vx", vx, self.MIN_VX, self.MAX_VX
        )

        self._require_range(
            "wz", wz, self.MIN_WZ, self.MAX_WZ
        )

        robot = self._require_connection()

        robot.rider_move_x(float(vx))
        robot.rider_turn(float(wz))

    def set_height(self, height_z: float) -> None:

        self._require_range(
            "height_z",
            height_z,
            self.MIN_HEIGHT,
            self.MAX_HEIGHT,
        )

        robot = self._require_connection()

        robot.rider_height(float(height_z))

    def perform_action(self, action_id: int) -> None:

        if not self.MIN_ACTION_ID <= action_id <= self.MAX_ACTION_ID:
            raise ValueError("Invalid Rider action ID")

        robot = self._require_connection()

        robot.rider_action(action_id)

    def read_telemetry(self) -> RobotTelemetry:

        robot = self._require_connection()

        telemetry = RobotTelemetry(
            pitch_deg=float(robot.rider_read_pitch()),
            roll_deg=float(robot.rider_read_roll()),
            yaw_deg=float(robot.rider_read_yaw()),
            battery_pct=float(robot.rider_read_battery()),
        )

        values = (
            telemetry.pitch_deg,
            telemetry.roll_deg,
            telemetry.yaw_deg,
            telemetry.battery_pct,
        )

        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("Invalid sensor data received")

        if not 0.0 <= telemetry.battery_pct <= 100.0:
            raise RuntimeError("Invalid battery measurement")

        return telemetry

    def request_safe_response(self) -> None:
        """
        Best-effort zero-velocity request.

        This is NOT a certified emergency stop.
        Physical behaviour must be verified on the actual robot.
        """

        robot = self._require_connection()

        logger.critical("Requesting zero-velocity response")

        try:
            robot.rider_move_x(0.0)
        finally:
            robot.rider_turn(0.0)

    def close(self) -> None:

        if self._robot is None:
            return

        robot = self._robot

        try:
            close_method = getattr(robot, "close", None)

            if callable(close_method):
                close_method()
            else:
                logger.warning(
                    "Installed xgolib exposes no verified close() "
                    "method; serial cleanup will rely on process exit"
                )

        finally:
            self._robot = None
