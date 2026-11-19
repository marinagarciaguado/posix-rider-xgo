"""Independent Python protocol codec and POSIX queues via ctypes (Linux)."""
import ctypes
import errno
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import unittest

BINARY = Path(__file__).resolve().parents[1] / "build" / "rider_bridge_sim"
FORMAT = struct.Struct("!IHHIIQQQQIIIIQQQQ")
assert FORMAT.size == 96
LIB = ctypes.CDLL("librt.so.1", use_errno=True)
LIB.mq_open.restype = ctypes.c_int
LIB.mq_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
LIB.mq_send.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint]
LIB.mq_receive.restype = ctypes.c_ssize_t
LIB.mq_receive.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t,
                         ctypes.POINTER(ctypes.c_uint)]
LIB.mq_close.argtypes = [ctypes.c_int]
LIB.mq_unlink.argtypes = [ctypes.c_char_p]


class Attr(ctypes.Structure):
    _fields_ = [(n, ctypes.c_long) for n in ("flags", "maxmsg", "msgsize", "curmsgs")] + [
        ("padding", ctypes.c_long * 4)
    ]


def now():
    return time.monotonic_ns() // 1_000_000


class BridgeProtocolTests(unittest.TestCase):
    def setUp(self):
        base = f"/rider_wire_{os.getpid()}_{time.monotonic_ns()}"
        self.names = [(base + suffix).encode() for suffix in ("_o", "_r")]
        self.queues = []
        self.addCleanup(self.cleanup)
        for name in self.names:
            attr = Attr(maxmsg=10, msgsize=96)
            q = LIB.mq_open(name, os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_NONBLOCK,
                            ctypes.c_uint(0o600), ctypes.byref(attr))
            if q < 0:
                raise OSError(ctypes.get_errno(), "mq_open")
            self.queues.append(q)
        self.p = subprocess.Popen([str(BINARY), *(n.decode() for n in self.names)])
        self.sequence = 0
        self.history = []
        self.wait_for(16)

    def cleanup(self):
        if hasattr(self, "p"):
            if self.p.poll() is None:
                self.p.send_signal(signal.SIGCONT)
                self.p.terminate()
            self.p.wait(timeout=3)
        for q in self.queues:
            LIB.mq_close(q)
        for name in self.names:
            LIB.mq_unlink(name)

    def send(self, type, gen=1, command=0, direction=0, duration=0, expires_ms=None, priority=10):
        self.sequence += 1
        t = now()
        data = FORMAT.pack(0x52445231, 1, 96, type, 0, self.sequence, gen, command,
                             t + 500 if expires_ms is None else expires_ms,
                             duration, direction + 1, 0, 0, 0, 0, t, 0)
        self.send_bytes(data, priority)

    def send_bytes(self, data, priority=10):
        if LIB.mq_send(self.queues[0], data, len(data), priority) != 0:
            raise OSError(ctypes.get_errno(), "mq_send")

    def receive(self):
        data = ctypes.create_string_buffer(96)
        priority = ctypes.c_uint()
        n = LIB.mq_receive(self.queues[1], data, 96, ctypes.byref(priority))
        if n < 0:
            if ctypes.get_errno() == errno.EAGAIN:
                return None
            raise OSError(ctypes.get_errno(), "mq_receive")
        self.assertEqual(n, 96)
        m = FORMAT.unpack(data.raw)
        self.assertEqual(m[:3], (0x52445231, 1, 96))
        self.history.append(m)
        return m

    def wait_for(self, type, timeout=1):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self.receive()
            if m is not None and m[3] == type:
                return m
            time.sleep(0.005)
        self.fail(f"Missing respuesta {type}: {self.history}")

    def arm_bridge(self):
        self.send(1)
        self.send(3, priority=20)
        self.wait_for(21)

    def test_stop_overtakes_old_generation_motion(self):
        self.p.send_signal(signal.SIGSTOP)
        for i in range(6):
            self.send(4, command=i + 1, direction=1, duration=100, priority=0)
        self.send(2, gen=2, priority=30)
        self.send(3, gen=2, priority=20)
        self.p.send_signal(signal.SIGCONT)
        stop_event = self.wait_for(20)
        self.assertEqual(stop_event[6], 2)
        self.assertEqual(stop_event[10], 1)  # Encoded direction: stopped
        self.wait_for(21)
        rejections = []
        for _ in range(6):
            rejections.append(self.wait_for(22))
        self.assertTrue(all(m[12] == 2 for m in rejections))  # REASON_GENERATION
        self.assertFalse(any(m[3] == 18 for m in self.history))

    def test_command_expired_in_transit_does_not_start(self):
        self.arm_bridge()
        self.send(4, command=1, direction=1, duration=100, expires_ms=now() - 1, priority=0)
        rejection = self.wait_for(22)
        self.assertEqual(rejection[12], 1)  # REASON_EXPIRED
        self.assertFalse(any(m[3] == 18 for m in self.history))

    def test_stop_invalidates_older_sequences_in_same_generation(self):
        self.arm_bridge()
        self.p.send_signal(signal.SIGSTOP)
        self.send(4, command=1, direction=1, duration=100, priority=0)
        self.send(2, priority=30)
        self.send(3, priority=20)
        self.p.send_signal(signal.SIGCONT)
        self.wait_for(20)
        self.wait_for(21)
        self.assertEqual(self.wait_for(22)[12], 8)
        self.assertFalse(any(m[3] == 18 for m in self.history))

    def test_watchdog_stops_without_heartbeats(self):
        self.arm_bridge()
        self.send(4, command=1, direction=1, duration=3000, priority=0)
        self.wait_for(18)
        fault = self.wait_for(23, timeout=1.5)
        self.assertEqual(fault[12], 6)  # REASON_WATCHDOG
        self.assertEqual(fault[10], 1)
        self.assertFalse(fault[4] & 1)

    def test_invalid_version_and_length_inhibit_motion(self):
        self.arm_bridge()
        data = bytearray(FORMAT.pack(0x52445231, 2, 96, 4, 0, 10, 1, 1,
                                      now() + 500, 100, 2, 0, 0, 0, 0, now(), 0))
        self.send_bytes(bytes(data), 0)
        fault = self.wait_for(23)
        self.assertEqual(fault[12], 8)  # REASON_PROTOCOL
        self.assertFalse(fault[4] & 1)
        self.send_bytes(b"short", 0)
        self.assertEqual(self.wait_for(23)[12], 8)


if __name__ == "__main__":
    unittest.main(verbosity=2)
