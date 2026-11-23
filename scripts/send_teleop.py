#!/usr/bin/env python3
"""Send v1 XGO teleoperation datagrams from a paired source port."""

import argparse
from enum import IntEnum
import math
import socket
import struct
import time

PACKET = struct.Struct("!4sBBHIii")
PERIOD_S = 0.05


class Opcode(IntEnum):
    HEARTBEAT = 1
    MOVE = 2
    HEIGHT = 3
    ESTOP = 4
    RESET = 5


class Operator:
    def __init__(self, sock, destination):
        self.sock = sock
        self.destination = destination
        self._last_sequence = 0

    def send(self, opcode, first=0, second=0):
        sequence = max(self._last_sequence + 1, time.monotonic_ns() // 1_000_000)
        self._last_sequence = sequence
        self.sock.sendto(
            PACKET.pack(b"XGO2", 1, opcode, 0, sequence & 0xFFFFFFFF, first, second),
            self.destination,
        )


def run(operator, args):
    if args.command == "estop":
        operator.send(Opcode.ESTOP)
        return

    start = time.monotonic()
    pending_reset = args.reset
    pending_height = args.command == "height"
    try:
        while time.monotonic() - start < args.seconds:
            operator.send(Opcode.HEARTBEAT)
            # Give the supervisor a safety cycle to observe operator liveness.
            elapsed = time.monotonic() - start
            if pending_reset and elapsed >= 2 * PERIOD_S:
                operator.send(Opcode.RESET)
                pending_reset = False
            if args.command == "move":
                operator.send(Opcode.MOVE, round(args.vx * 1000), round(args.wz * 1000))
            if pending_height and elapsed >= 5 * PERIOD_S and not pending_reset:
                operator.send(Opcode.HEIGHT, args.height)
                pending_height = False
            time.sleep(PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        if args.command == "move":
            operator.send(Opcode.MOVE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("idle", "move", "height", "estop"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=45800)
    parser.add_argument("--source-ip", default="0.0.0.0")
    parser.add_argument("--source-port", type=int, default=45801)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--height", type=int, default=90)
    parser.add_argument("--reset", action="store_true", help="request explicit recovery once")
    args = parser.parse_args()
    if not (0 < args.port < 65536 and 0 < args.source_port < 65536):
        parser.error("ports must be between 1 and 65535")
    if not math.isfinite(args.seconds) or args.seconds <= 5 * PERIOD_S:
        parser.error("--seconds must be finite and greater than 0.25")
    if not (-0.30 <= args.vx <= 0.30 and -0.50 <= args.wz <= 0.50):
        parser.error("velocity exceeds supervisor limits")
    if not 60 <= args.height <= 120:
        parser.error("height must be between 60 and 120 mm")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.source_ip, args.source_port))
        run(Operator(sock, (args.host, args.port)), args)


if __name__ == "__main__":
    main()
