"""Record repeatable scheduling and fault runs against the mock supervisor."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import platform
import signal
import socket
import statistics
import subprocess
import sys
import time

from scripts.send_teleop import Opcode, Operator


SCENARIOS = ("baseline", "normal", "stress", "operator-loss", "telemetry-loss", "bridge-crash")


def child_pid(events):
    if not events.exists():
        return None
    for line in events.read_text(encoding="utf-8").splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if event["event"] == "connected":
            return event["pid"]
    return None


def cpu_ticks():
    fields = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()
    ticks = list(map(int, fields[1:9]))
    return sum(ticks), ticks[3] + ticks[4]


def terminate(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    return process.returncode


def summarize(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    jitter = sorted(abs(int(row["jitter_ns"])) for row in rows
                    if int(row["observed_period_ns"]) > 0)
    latency = [int(row["emergency_dispatch_latency_ns"]) for row in rows
               if int(row["emergency_dispatch_latency_ns"]) >= 0]
    return {
        "cycles_measured": len(jitter),
        "median_absolute_jitter_ns": statistics.median(jitter) if jitter else None,
        "p95_absolute_jitter_ns": jitter[math.ceil(len(jitter) * 0.95) - 1] if jitter else None,
        "max_absolute_jitter_ns": max(jitter, default=None),
        "deadline_misses": sum(int(row["deadline_miss"]) for row in rows),
        "timer_overruns": sum(int(row["timer_overrun_count"]) for row in rows),
        "max_emergency_dispatch_latency_ns": max(latency, default=None),
        "faults": sorted({row["fault_code"] for row in rows if row["fault_code"] != "none"}),
    }


def operate(process, operator, events, scenario, seconds, log_path):
    deadline = time.monotonic() + 5
    pid = None
    while time.monotonic() < deadline and process.poll() is None:
        pid = child_pid(events)
        if pid is not None and "Bridge READY with valid telemetry" in log_path.read_text(
                encoding="utf-8"):
            break
        time.sleep(0.02)
    else:
        raise RuntimeError(f"Mock did not become ready. See {log_path}")

    started = time.monotonic()
    fault_injected = False
    paused = False
    try:
        while process.poll() is None:
            elapsed = time.monotonic() - started
            if elapsed >= seconds:
                break
            if elapsed >= seconds / 2 and not fault_injected:
                if scenario == "bridge-crash":
                    os.kill(pid, signal.SIGKILL)
                elif scenario == "telemetry-loss":
                    os.kill(pid, signal.SIGSTOP)
                    paused = True
                fault_injected = True
            if scenario != "baseline" and not (scenario == "operator-loss" and fault_injected):
                operator.send(Opcode.HEARTBEAT)
                operator.send(Opcode.MOVE, 100)
            time.sleep(0.05)
    finally:
        if paused and process.poll() is None:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass


def run_one(args, folder, policy, scenario):
    metrics = folder / "metrics.csv"
    events = folder / "events.jsonl"
    log_path = folder / "supervisor.log"
    environment = os.environ | {"XGO_MOCK_EVENTS": str(events.resolve())}
    environment.pop("XGO_MOCK_TELEMETRY", None)
    busy = []
    cpu_before, idle_before = cpu_ticks()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.bind(("127.0.0.1", 0))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            destination = receiver.getsockname()
        operator = Operator(sender, destination)
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                ["bin/xgo_supervisor", "--mock", "--scheduler", policy,
                 "--port", str(destination[1]), "--operator-port", str(sender.getsockname()[1]),
                 "--python", sys.executable, "--metrics", str(metrics)],
                stdout=log, stderr=log, env=environment,
            )
        try:
            if scenario == "stress":
                for _ in range(args.stress_workers):
                    busy.append(subprocess.Popen([sys.executable, "-c", "while True: pass"]))
            operate(process, operator, events, scenario, args.seconds, log_path)
        finally:
            result = terminate(process)
            for worker in busy:
                terminate(worker)

    cpu_total, idle_total = cpu_ticks()
    elapsed_ticks = cpu_total - cpu_before
    governor = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    scheduler_line = next((line for line in log_path.read_text(encoding="utf-8").splitlines()
                           if line.startswith("Safety scheduler:")), None)
    expected_exit = result != 0 if scenario == "bridge-crash" else result == 0
    if scheduler_line is None or not expected_exit:
        raise RuntimeError(f"Unexpected supervisor result {result}. See {log_path}")
    report = {
        "platform": platform.platform(),
        "cpu_governor": governor.read_text().strip() if governor.exists() else None,
        "requested_scheduler": policy,
        "effective_scheduler": scheduler_line.removeprefix("Safety scheduler: "),
        "scenario": scenario,
        "configured_seconds": args.seconds,
        "stress_workers": len(busy),
        "overall_cpu_utilization": (
            1 - (idle_total - idle_before) / elapsed_ticks if elapsed_ticks > 0 else None
        ),
        "supervisor_exit_code": result,
        **summarize(metrics),
    }
    (folder / "summary.json").write_text(json.dumps(report, indent=2) + chr(10), encoding="utf-8")
    print(f"{folder}  {report['cycles_measured']} cycles  "
          f"p95 jitter {report['p95_absolute_jitter_ns']} ns")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=(*SCENARIOS, "all"), default="all")
    parser.add_argument("--scheduler", choices=("other", "fifo", "both"), default="other")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--stress-workers", type=int, default=2)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--output", type=Path, default=Path("logs/benchmarks"))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 1:
        parser.error("--seconds must be finite and at least 1")
    if args.stress_workers < 1 or args.repeat < 1:
        parser.error("--stress-workers and --repeat must be positive")
    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    policies = ("other", "fifo") if args.scheduler == "both" else (args.scheduler,)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    for policy in policies:
        for scenario in scenarios:
            for run in range(1, args.repeat + 1):
                folder = args.output / f"{stamp}-{policy}-{scenario}-{run}"
                folder.mkdir(parents=True)
                run_one(args, folder, policy, scenario)


if __name__ == "__main__":
    main()
