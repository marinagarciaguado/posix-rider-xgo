"""End-to-end tests using real Linux UDP and POSIX message queues."""

import csv
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest

import posix_ipc

from scripts.send_teleop import PACKET, Operator, Opcode


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "bin" / "xgo_supervisor"
QUEUES = ("/xgo_cmd_mq", "/xgo_telem_mq")


@unittest.skipUnless(sys.platform == "linux" and SUPERVISOR.is_file(),
                     "build the Linux supervisor with make")
class TestSupervisor(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="xgo-test-")
        self.addCleanup(self.temp.cleanup)
        folder = Path(self.temp.name)
        self.events = folder / "events.jsonl"
        self.telemetry = folder / "telemetry.json"
        self.metrics = folder / "metrics.csv"
        self.log = folder / "supervisor.log"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.sock.close)
        self.sock.bind(("127.0.0.1", 0))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port:
            port.bind(("127.0.0.1", 0))
            self.destination = port.getsockname()
        self.operator = Operator(self.sock, self.destination)

    def launch(self, *options, python=sys.executable):
        for name in QUEUES:
            try:
                queue = posix_ipc.MessageQueue(name)
            except posix_ipc.ExistentialError:
                continue
            queue.close()
            self.fail(f"Existing queue {name}; an integration test must have exclusive use")

        environment = os.environ | {
            "XGO_MOCK_EVENTS": str(self.events),
            "XGO_MOCK_TELEMETRY": str(self.telemetry),
        }
        with self.log.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                [SUPERVISOR, "--mock", "--port", str(self.destination[1]),
                 "--operator-port", str(self.sock.getsockname()[1]), "--python", python,
                 "--metrics", self.metrics, *options],
                cwd=ROOT, env=environment, stdout=log, stderr=log,
            )
        self.addCleanup(self.stop_if_running, process)
        return process

    @staticmethod
    def stop_if_running(process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def wait_for(self, predicate, pulse=True):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if predicate():
                return
            if pulse:
                self.operator.send(Opcode.HEARTBEAT)
            time.sleep(0.025)
        self.fail(self.log.read_text(encoding="utf-8"))

    def rows(self):
        if not self.metrics.exists():
            return []
        with self.metrics.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))

    def current_state(self):
        rows = self.rows()
        return (rows[-1]["safety_state"], rows[-1]["fault_code"]) if rows else (None, None)

    def event_rows(self):
        if not self.events.exists():
            return []
        return [json.loads(line) for line in self.events.read_text(encoding="utf-8").splitlines()]

    def event_count(self, event):
        return sum(row["event"] == event for row in self.event_rows())

    def set_telemetry(self, **values):
        pending = self.telemetry.with_suffix(".tmp")
        pending.write_text(json.dumps(values), encoding="utf-8")
        pending.replace(self.telemetry)

    def assert_queues_removed(self):
        for name in QUEUES:
            with self.assertRaises(posix_ipc.ExistentialError):
                posix_ipc.MessageQueue(name)

    def test_motion_tilt_recovery_and_operator_timeout(self):
        process = self.launch()
        self.wait_for(lambda: self.current_state() == ("STARTING", "none"), pulse=False)
        self.operator.send(Opcode.MOVE, 100)
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        self.assertEqual(self.event_count("move"), 0)

        self.operator.send(Opcode.MOVE, -100, 125)
        self.wait_for(lambda: self.event_count("move") == 1)
        movement = next(row for row in self.event_rows() if row["event"] == "move")
        self.assertAlmostEqual(movement["vx"], -0.1)
        self.assertAlmostEqual(movement["wz"], 0.125)
        self.set_telemetry(pitch_deg=22)
        self.wait_for(lambda: self.current_state() == ("EMERGENCY", "tilt"))
        self.wait_for(lambda: self.event_count("safe_response") >= 1)

        self.operator.send(Opcode.MOVE, 100)
        self.operator.send(Opcode.RESET)
        keepalive_until = time.monotonic() + 0.10
        self.wait_for(lambda: time.monotonic() >= keepalive_until)
        self.assertEqual(self.current_state(), ("EMERGENCY", "tilt"))
        self.assertEqual(self.event_count("move"), 1)

        self.set_telemetry()
        self.wait_for(lambda: self.rows() and self.rows()[-1]["pitch_deg"] == "0.000")
        self.assertEqual(self.current_state(), ("EMERGENCY", "tilt"))
        self.operator.send(Opcode.RESET)
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        self.operator.send(Opcode.HEIGHT, 90)
        self.wait_for(lambda: self.event_count("height") == 1)
        self.wait_for(lambda: self.current_state() == ("EMERGENCY", "operator_timeout"),
                      pulse=False)

        process.terminate()
        self.assertEqual(process.wait(timeout=4), 0)
        self.assertEqual(self.event_count("closed"), 1)
        self.assert_queues_removed()

    def test_child_death_is_detected_and_resources_are_reaped(self):
        process = self.launch()
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        child = next(row["pid"] for row in self.event_rows() if row["event"] == "connected")
        os.kill(child, signal.SIGKILL)
        self.assertNotEqual(process.wait(timeout=4), 0)
        self.assertIn("Bridge exited on signal", self.log.read_text(encoding="utf-8"))
        self.assert_queues_removed()

    def test_exec_failure_cleans_up_queues(self):
        process = self.launch(python="/missing/xgo-python-executable")
        self.assertNotEqual(process.wait(timeout=4), 0)
        self.assertIn("127", self.log.read_text(encoding="utf-8"))
        self.assert_queues_removed()

    def test_telemetry_only_has_no_motion_or_stop_writes(self):
        process = self.launch("--telemetry-only")
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        self.operator.send(Opcode.MOVE, 100)
        self.operator.send(Opcode.HEIGHT, 90)
        self.operator.send(Opcode.ESTOP)
        self.wait_for(lambda: self.current_state() == ("EMERGENCY", "operator_estop"))
        process.terminate()
        self.assertEqual(process.wait(timeout=4), 0)
        self.assertEqual([row["event"] for row in self.event_rows()], ["connected", "closed"])
        self.assert_queues_removed()

    def test_udp_rejects_unpaired_malformed_and_out_of_range_packets(self):
        process = self.launch()
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as unpaired:
            sender = Operator(unpaired, self.destination)
            sender.send(Opcode.MOVE, 100)
        self.sock.sendto(b"XGO2", self.destination)
        self.operator.send(Opcode.MOVE, 301)
        self.operator.send(Opcode.MOVE, 100)
        self.wait_for(lambda: self.event_count("move") >= 1)
        duplicate = self.operator._last_sequence & 0xFFFFFFFF
        packet = PACKET.pack(b"XGO2", 1, Opcode.MOVE, 0, duplicate, 100, 0)
        self.sock.sendto(packet, self.destination)
        keepalive_until = time.monotonic() + 0.10
        self.wait_for(lambda: time.monotonic() >= keepalive_until)
        self.operator.send(Opcode.ESTOP)
        self.wait_for(lambda: self.current_state() == ("EMERGENCY", "operator_estop"))
        self.assertEqual(self.event_count("move"), 1)
        process.terminate()
        self.assertEqual(process.wait(timeout=4), 0)
        self.assert_queues_removed()

    def test_python_stops_when_supervisor_heartbeat_is_lost(self):
        process = self.launch()
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        os.kill(process.pid, signal.SIGSTOP)
        try:
            self.wait_for(lambda: self.event_count("safe_response") >= 1, pulse=False)
            self.assertIn("C supervisor heartbeat timeout", self.log.read_text(encoding="utf-8"))
        finally:
            os.kill(process.pid, signal.SIGCONT)
        process.terminate()
        self.assertEqual(process.wait(timeout=4), 0)
        self.assert_queues_removed()

    def test_invalid_sensor_and_low_battery_faults(self):
        process = self.launch()
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        self.set_telemetry(fail=True)
        self.wait_for(lambda: self.current_state() == ("EMERGENCY", "invalid_telemetry"))
        self.set_telemetry()
        self.wait_for(lambda: self.rows() and
                      int(self.rows()[-1]["emergency_dispatch_latency_ns"]) >= 0 and
                      self.rows()[-1]["sensor_valid"] == "1")
        self.operator.send(Opcode.RESET)
        self.wait_for(lambda: self.current_state() == ("NORMAL", "none"))
        self.set_telemetry(battery_pct=10)
        self.wait_for(lambda: self.current_state() == ("EMERGENCY", "low_battery"))
        process.terminate()
        self.assertEqual(process.wait(timeout=4), 0)
        self.assert_queues_removed()
