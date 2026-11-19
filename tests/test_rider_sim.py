"""Linux integration tests using only the Python standard library."""
import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

BINARY = Path(__file__).resolve().parents[1] / "build" / "rider_sim"


class Simulator(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.csv = Path(self.tmp.name) / "events.csv"
        self.output = tempfile.TemporaryFile(mode="w+")
        self.p = subprocess.Popen(
            [str(BINARY), str(self.csv)], stdin=subprocess.PIPE,
            stdout=self.output, stderr=subprocess.STDOUT, text=True,
        )

    def tearDown(self):
        if self.p.poll() is None:
            self.p.kill()
        self.p.wait(timeout=3)
        if self.p.stdin and not self.p.stdin.closed:
            self.p.stdin.close()
        self.output.close()
        self.tmp.cleanup()

    def send(self, text):
        self.p.stdin.write(text + "\n")
        self.p.stdin.flush()

    def events(self):
        if not self.csv.exists():
            return []
        with self.csv.open() as f:
            return [r for r in csv.DictReader(f) if all(v is not None for v in r.values())]

    def wait_for(self, name, count=1, timeout=5):
        limit = time.monotonic() + timeout
        while time.monotonic() < limit:
            rows = [r for r in self.events() if r["event"] == name]
            if len(rows) >= count:
                return rows[-1]
            if self.p.poll() is not None:
                break
            time.sleep(0.01)
        self.output.seek(0)
        self.fail(f"Missing {name} ({count}):\n{self.output.read()}")

    def request_shutdown(self, eof=False):
        if eof:
            self.p.stdin.close()
        else:
            self.send("quit")
        self.assertEqual(self.p.wait(timeout=3), 0)
        self.assertEqual(self.events()[-1]["event"], "shutdown_completed")
        self.output.seek(0)
        output = self.output.read()
        self.assertIn("4 threads joined", output)
        self.assertIn("dropped events=0", output)
        self.assert_cleanup()

    def assert_cleanup(self):
        self.assertEqual(list(Path("/dev/mqueue").glob(f"rider_{self.p.pid}_*")), [])
        children = [r for r in self.events() if r["event"] == "bridge_started"]
        if children:
            self.assertFalse(Path(f"/proc/{children[0]['id']}").exists(), "Child was not reaped")

    def test_timed_motion_in_both_directions(self):
        self.send("forward 100")
        self.wait_for("rejected_latched")
        self.send("arm\nforward 100\nbackward 100")
        self.wait_for("movement_completed", 2)
        starts = [r for r in self.events() if r["event"] == "movement_started"]
        self.assertEqual([r["direction"] for r in starts], ["1", "-1"])
        completions = [r for r in self.events() if r["event"] == "movement_completed"]
        for start_ms, end in zip(starts, completions):
            self.assertGreaterEqual(int(end["bridge_timestamp_ms"]) - int(start_ms["bridge_timestamp_ms"]), 100)
            self.assertEqual(end["direction"], "0")
        self.request_shutdown()

    def test_stop_with_full_queue_and_no_automatic_resume(self):
        self.send("arm\nforward 3000")
        self.wait_for("movement_started")
        self.send("forward 100\n" * 12)
        self.wait_for("rejected_queue_full")
        self.send("stop")
        stop_event = self.wait_for("stop_requested")
        confirmation = self.wait_for("stop_confirmed")
        self.assertEqual(stop_event["pending"], "0")
        self.assertEqual(confirmation["direction"], "0")
        self.send("forward 100")
        self.wait_for("rejected_latched")
        self.send("arm\nstate")
        state = self.wait_for("state_armed")
        self.assertEqual(state["direction"], "0")
        self.assertEqual(state["pending"], "0")
        self.assertEqual(len([r for r in self.events() if r["event"] == "movement_started"]), 1)
        self.send("backward 80")
        self.wait_for("movement_completed")
        starts = [r for r in self.events() if r["event"] == "movement_started"]
        self.assertEqual(len(starts), 2)
        self.assertNotEqual(starts[0]["generation"], starts[1]["generation"])
        self.request_shutdown()

    def test_expired_command_is_not_executed(self):
        self.send("arm\nforward 2500")
        self.wait_for("movement_started")
        self.send("backward 100")
        self.wait_for("command_discarded")
        self.assertEqual(len([r for r in self.events() if r["event"] == "movement_started"]), 1)
        self.request_shutdown()

    def test_invalid_input_and_eof(self):
        self.send("arm\nforward 0\nforward -1\nforward 3001\nforward 9999999999999999999999\nforward 12x")
        self.wait_for("invalid_duration", 5)
        self.send("desconocido\nstop extra\nforward 10 extra")
        self.wait_for("invalid_command", 3)
        self.send("x" * 200)
        self.wait_for("line_too_long")
        self.request_shutdown(eof=True)

    def test_eof_during_motion(self):
        self.send("arm\nforward 3000")
        self.wait_for("movement_started")
        self.request_shutdown(eof=True)
        self.assertIn("stop_confirmed", [r["event"] for r in self.events()])

    def test_sigint_without_input(self):
        self.send("state")
        self.wait_for("state_latched")
        self.p.send_signal(signal.SIGINT)
        self.assertEqual(self.p.wait(timeout=3), 0)
        self.assertIn("signal_received", [r["event"] for r in self.events()])
        self.assertEqual(self.events()[-1]["event"], "shutdown_completed")

    def test_sigterm_during_motion(self):
        self.send("arm\nforward 3000")
        self.wait_for("movement_started")
        self.p.send_signal(signal.SIGTERM)
        self.assertEqual(self.p.wait(timeout=3), 0)
        self.assertIn("stop_confirmed", [r["event"] for r in self.events()])
        self.assertEqual(self.events()[-1]["event"], "shutdown_completed")

    def check_fault_and_recovery(self, mode, name):
        self.send("arm\nforward 3000")
        self.wait_for("movement_started")
        self.send("backward 100\nsensor " + mode)
        fault = self.wait_for(name)
        confirmation = self.wait_for("stop_confirmed")
        self.assertEqual(fault["pending"], "0")
        self.assertEqual(confirmation["direction"], "0")
        self.assertGreaterEqual(int(confirmation["stop_latency_us"]), 0)
        if mode == "invalid":
            self.assertEqual(fault["valid"], "0")
        else:
            self.assertGreater(int(fault["sample_age_ms"]), 600)
        self.send("arm")
        self.wait_for("arm_rejected_telemetry")
        self.send("sensor normal")
        self.wait_for("telemetry_recovered")
        self.send("forward 100\nstate")
        self.wait_for("rejected_latched")
        self.wait_for("state_latched")
        self.assertEqual(len([r for r in self.events() if r["event"] == "movement_started"]), 1)
        self.send("arm\nbackward 80")
        self.wait_for("movement_completed")
        starts = [r for r in self.events() if r["event"] == "movement_started"]
        self.assertEqual(len(starts), 2)
        self.assertEqual(starts[-1]["direction"], "-1")
        self.assertNotEqual(starts[0]["generation"], starts[-1]["generation"])
        self.request_shutdown()

    def test_invalid_telemetry_stops_and_requires_arm(self):
        self.check_fault_and_recovery("invalid", "invalid_telemetry_fault")

    def test_frozen_telemetry_stops_and_requires_arm(self):
        self.check_fault_and_recovery("frozen", "stale_telemetry_fault")

    def test_timer_metrics_and_scheduling(self):
        self.wait_for("supervision_cycle", 3)
        self.send("load 120")
        # Measure accumulated expirations when the next timer notification is consumed.
        limit = time.monotonic() + 5
        while time.monotonic() < limit:
            cycles = [r for r in self.events() if r["event"] == "supervision_cycle"]
            if any(int(r["missed_expirations"]) > 0 for r in cycles):
                break
            time.sleep(0.01)
        else:
            self.fail("Injected load did not produce timer overruns")
        self.assertTrue(any(int(r["lateness_us"]) >= 100000 for r in cycles))
        self.assertTrue(all(int(r["lateness_us"]) >= 0 for r in cycles))
        self.assertTrue(all(int(r["thread_cpu_us"]) >= 0 for r in cycles))
        self.assertTrue(any(int(r["interval_us"]) > 0 for r in cycles))
        scheduling = [r for r in self.events() if r["event"].startswith("scheduling_")]
        self.assertEqual(len(scheduling), 4)
        self.assertTrue(all(int(r["policy"]) >= 0 for r in scheduling))
        self.assertTrue(all(int(r["priority"]) >= 0 for r in scheduling))
        self.request_shutdown()

    def test_child_death_does_not_confirm_stop(self):
        self.send("arm\nforward 3000")
        self.wait_for("movement_started")
        child = int(self.wait_for("bridge_started")["id"])
        os.kill(child, signal.SIGKILL)
        self.assertNotEqual(self.p.wait(timeout=4), 0)
        self.assertIn("bridge_exited_unexpectedly", [r["event"] for r in self.events()])
        self.assertNotIn("stop_confirmed", [r["event"] for r in self.events()])
        self.assert_cleanup()

    def test_stalled_child_does_not_block_shutdown(self):
        self.send("arm\nforward 3000")
        self.wait_for("movement_started")
        child = int(self.wait_for("bridge_started")["id"])
        os.kill(child, signal.SIGSTOP)
        self.assertNotEqual(self.p.wait(timeout=4), 0)
        self.assertIn("bridge_unresponsive", [r["event"] for r in self.events()])
        self.assertNotIn("stop_confirmed", [r["event"] for r in self.events()])
        self.assert_cleanup()

    def test_exec_failure_cleans_up_queues(self):
        self.request_shutdown()
        p = subprocess.Popen([str(BINARY), str(self.csv), "/missing/bridge"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        _, error = p.communicate(timeout=4)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("bridge startup", error)
        self.assertEqual(list(Path("/dev/mqueue").glob(f"rider_{p.pid}_*")), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
