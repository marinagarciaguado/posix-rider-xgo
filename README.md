# XGO Rider 2 POSIX supervisor

The Linux C11 supervisor handles operator input, command arbitration, periodic safety checks, process supervision, and timing metrics. A Python child owns the hardware interface and exchanges commands and telemetry over two POSIX message queues. The ESP32 retains its factory balancing and motor control.

## Build and run with the mock

Run from the project root on little-endian Linux. Use GCC, Make, and Python 3.10 or newer with POSIX message queue support. If pip builds the IPC dependency from source, install the development headers for the chosen Python version.

```sh
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements.txt
make
make test
bin/xgo_supervisor --mock
```

In a second terminal, send teleoperation to the mock.

```sh
python3 scripts/send_teleop.py move --vx 0.10 --seconds 3
python3 scripts/send_teleop.py idle --reset --seconds 2
```

Mock mode is the default. The receiver binds to loopback on UDP port 45800 and accepts a single paired source on port 45801. Set `--bind-ip`, `--port`, `--operator-ip`, and `--operator-port` together with the sender's corresponding options for a dedicated, trusted operator network. IP and port pairing identifies the sender and provides no cryptographic authentication. Run only one sender at a time. Its monotonic sequence base survives sender process restarts on the same machine. Restart the supervisor after the operator machine reboots or changes.

Use `Ctrl+C` or SIGTERM to shut down. The supervisor releases its queues after the child exits and refuses to take over preexisting queues. If the supervisor was force-killed, verify that both old processes have exited, then remove only the abandoned `/xgo_cmd_mq` and `/xgo_telem_mq` queues before restarting. The process owner can use `posix_ipc.unlink_message_queue()` for each abandoned name.

## Runtime and wire contracts

- **C workers** receive UDP, evaluate safety at a nominal 50 Hz, and dispatch IPC. The dispatcher is the sole C-side command MQ writer and receives telemetry. The safety worker uses short snapshot critical sections and sends samples to a separate CSV logger. `config/robot_config.h` defines the initial software limits and periods. Bridge timing and recovery settings are supplied at launch by `src/process_manager.c`.
- **UDP v1** uses network byte order with the layout `!4sBBHIii`. The fields are magic `XGO2`, version `1`, opcode, zero reserved field, unsigned 32-bit sequence, and two signed 32-bit arguments. Opcodes are HEARTBEAT (1), MOVE (2), HEIGHT (3), ESTOP (4), and RESET (5). MOVE carries the two Rider SDK command values scaled by 1000, HEIGHT carries millimetres in the first argument, and unused arguments are zero. Verify units, quantization, and axis signs on the installed SDK before commanding real motion. Sequences use half-range modular comparison. Only a valid explicit heartbeat renews operator liveness.
- **Local IPC v1** uses 32-byte little-endian records and IEEE-754 binary32 floats. The C header and Python types specify the complete layout. Commands use the Pi's monotonic time at ingress or creation. Telemetry carries monotonic acquisition time, and C also records its receipt time. The dispatcher reserves space for priority ESTOP and bounds normal MQ backlog. The Python receiver tracks ESTOP sequences separately because MQ priority can reorder them ahead of older normal messages.
- **Safety and recovery** require valid READY telemetry and a current operator heartbeat before motion. The ring invalidates older work by generation and discards stale commands by age. On a fault, the dispatcher prioritizes ESTOP. RESET requires fresh valid telemetry, a current operator heartbeat, and stricter recovery limits. C waits for fresh READY feedback after RESET before admitting new motion. The Python bridge has a separate command lease for nonzero movement and a supervisor heartbeat watchdog. Recovery after a latched supervisor heartbeat failure requires restarting the processes.

## Tests and measurements

`make test` runs the C queue concurrency test, Python unit tests, and Linux integration tests with real UDP and POSIX queues. The integration tests need exclusive use of the two queue names.

```sh
python3 -m scripts.run_benchmarks --scheduler both --repeat 2
```

The benchmark uses the mock for baseline, normal traffic, CPU load, operator loss, telemetry loss, and bridge termination. It records raw metrics and a summary in `logs/benchmarks/`. Inspect the effective scheduler in each report, since requesting FIFO may fall back when the process lacks permissions. The CSV tracks period, jitter, overruns, faults, and fault-to-MQ dispatch latency. The deadline criterion is an interval more than 2 ms above the target period or a timer overrun. Timing measurements characterize soft real-time behavior.

Set `XGO_MOCK_TELEMETRY` to a JSON file and update it by atomic replacement to inject `pitch_deg`, `roll_deg`, `yaw_deg`, `battery_pct`, or `fail: true`. Set `XGO_MOCK_EVENTS` to collect mock commands and stop requests as JSON lines.

## Hardware bring-up

Use the vendor-provided `xgolib` for the installed Rider 2 image. The adapter uses the Rider alias and the SDK's connection defaults. Set `XGO_UART_PORT` or `XGO_UART_BAUD` only after confirming the values supported by the installed SDK. Start with `bin/xgo_supervisor --hardware --telemetry-only`. This mode reads telemetry through the normal bridge and suppresses application actuator calls, including zero-velocity writes on shutdown. Check stationary readings, axis conventions, battery data, and SDK call duration before enabling motion.

The software ESTOP requests zero linear and turn commands through the SDK. Its result depends on the SDK, UART, and ESP32 still responding. Verify firmware behavior on lost targets and the independent physical stop procedure on a secured bench before enabling movement. Measure physical stopping time separately from IPC dispatch time. Start motion-enabled mode with `--hardware` after those checks.

[Rider 2 SDK reference](https://wiki.xgorobot.com/kb/the-xgo-series-robot/cmtimapdg000ku8uy6aik5qil?model=rider2)
