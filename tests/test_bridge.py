"""Protocolo independiente en Python y colas POSIX via ctypes (Linux)."""
import ctypes
import errno
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import unittest

BINARIO = Path(__file__).resolve().parents[1] / "build" / "rider_bridge_sim"
FORMATO = struct.Struct("!IHHIIQQQQIIIIQQQQ")
assert FORMATO.size == 96
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


def ahora():
    return time.monotonic_ns() // 1_000_000


class PuenteDirecto(unittest.TestCase):
    def setUp(self):
        base = f"/rider_wire_{os.getpid()}_{time.monotonic_ns()}"
        self.nombres = [(base + sufijo).encode() for sufijo in ("_o", "_r")]
        self.colas = []
        self.addCleanup(self.limpiar)
        for nombre in self.nombres:
            attr = Attr(maxmsg=10, msgsize=96)
            q = LIB.mq_open(nombre, os.O_CREAT | os.O_EXCL | os.O_RDWR | os.O_NONBLOCK,
                            ctypes.c_uint(0o600), ctypes.byref(attr))
            if q < 0:
                raise OSError(ctypes.get_errno(), "mq_open")
            self.colas.append(q)
        self.p = subprocess.Popen([str(BINARIO), *(n.decode() for n in self.nombres)])
        self.secuencia = 0
        self.historial = []
        self.esperar(16)

    def limpiar(self):
        if hasattr(self, "p"):
            if self.p.poll() is None:
                self.p.send_signal(signal.SIGCONT)
                self.p.terminate()
            self.p.wait(timeout=3)
        for q in self.colas:
            LIB.mq_close(q)
        for nombre in self.nombres:
            LIB.mq_unlink(nombre)

    def enviar(self, tipo, gen=1, orden=0, direccion=0, duracion=0, caduca=None, prioridad=10):
        self.secuencia += 1
        t = ahora()
        datos = FORMATO.pack(0x52445231, 1, 96, tipo, 0, self.secuencia, gen, orden,
                             t + 500 if caduca is None else caduca,
                             duracion, direccion + 1, 0, 0, 0, 0, t, 0)
        self.enviar_bytes(datos, prioridad)

    def enviar_bytes(self, datos, prioridad=10):
        if LIB.mq_send(self.colas[0], datos, len(datos), prioridad) != 0:
            raise OSError(ctypes.get_errno(), "mq_send")

    def recibir(self):
        datos = ctypes.create_string_buffer(96)
        prioridad = ctypes.c_uint()
        n = LIB.mq_receive(self.colas[1], datos, 96, ctypes.byref(prioridad))
        if n < 0:
            if ctypes.get_errno() == errno.EAGAIN:
                return None
            raise OSError(ctypes.get_errno(), "mq_receive")
        self.assertEqual(n, 96)
        m = FORMATO.unpack(datos.raw)
        self.assertEqual(m[:3], (0x52445231, 1, 96))
        self.historial.append(m)
        return m

    def esperar(self, tipo, plazo=1):
        fin = time.monotonic() + plazo
        while time.monotonic() < fin:
            m = self.recibir()
            if m is not None and m[3] == tipo:
                return m
            time.sleep(0.005)
        self.fail(f"No aparece respuesta {tipo}: {self.historial}")

    def armar(self):
        self.enviar(1)
        self.enviar(3, prioridad=20)
        self.esperar(21)

    def test_parada_adelanta_movimientos_de_generacion_antigua(self):
        self.p.send_signal(signal.SIGSTOP)
        for i in range(6):
            self.enviar(4, orden=i + 1, direccion=1, duracion=100, prioridad=0)
        self.enviar(2, gen=2, prioridad=30)
        self.enviar(3, gen=2, prioridad=20)
        self.p.send_signal(signal.SIGCONT)
        parada = self.esperar(20)
        self.assertEqual(parada[6], 2)
        self.assertEqual(parada[10], 1)  # direccion codificada: cero
        self.esperar(21)
        descartes = []
        for _ in range(6):
            descartes.append(self.esperar(22))
        self.assertTrue(all(m[12] == 2 for m in descartes))  # MOTIVO_GENERACION
        self.assertFalse(any(m[3] == 18 for m in self.historial))

    def test_orden_caducada_en_transito_no_inicia(self):
        self.armar()
        self.enviar(4, orden=1, direccion=1, duracion=100, caduca=ahora() - 1, prioridad=0)
        descarte = self.esperar(22)
        self.assertEqual(descarte[12], 1)  # MOTIVO_CADUCADO
        self.assertFalse(any(m[3] == 18 for m in self.historial))

    def test_parada_invalida_secuencias_previas_incluso_en_misma_generacion(self):
        self.armar()
        self.p.send_signal(signal.SIGSTOP)
        self.enviar(4, orden=1, direccion=1, duracion=100, prioridad=0)
        self.enviar(2, prioridad=30)
        self.enviar(3, prioridad=20)
        self.p.send_signal(signal.SIGCONT)
        self.esperar(20)
        self.esperar(21)
        self.assertEqual(self.esperar(22)[12], 8)
        self.assertFalse(any(m[3] == 18 for m in self.historial))

    def test_watchdog_detiene_sin_latidos(self):
        self.armar()
        self.enviar(4, orden=1, direccion=1, duracion=3000, prioridad=0)
        self.esperar(18)
        fallo = self.esperar(23, plazo=1.5)
        self.assertEqual(fallo[12], 6)  # MOTIVO_WATCHDOG
        self.assertEqual(fallo[10], 1)
        self.assertFalse(fallo[4] & 1)

    def test_version_y_tamano_invalidos_inhiben_movimiento(self):
        self.armar()
        datos = bytearray(FORMATO.pack(0x52445231, 2, 96, 4, 0, 10, 1, 1,
                                      ahora() + 500, 100, 2, 0, 0, 0, 0, ahora(), 0))
        self.enviar_bytes(bytes(datos), 0)
        fallo = self.esperar(23)
        self.assertEqual(fallo[12], 8)  # MOTIVO_PROTOCOLO
        self.assertFalse(fallo[4] & 1)
        self.enviar_bytes(b"corto", 0)
        self.assertEqual(self.esperar(23)[12], 8)


if __name__ == "__main__":
    unittest.main(verbosity=2)
