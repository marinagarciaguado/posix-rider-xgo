"""Pruebas de integracion en Linux; solo biblioteca estandar de Python."""
import csv
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

BINARIO = Path(__file__).resolve().parents[1] / "build" / "rider_sim"


class Simulador(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.csv = Path(self.tmp.name) / "eventos.csv"
        self.salida = tempfile.TemporaryFile(mode="w+")
        self.p = subprocess.Popen(
            [str(BINARIO), str(self.csv)], stdin=subprocess.PIPE,
            stdout=self.salida, stderr=subprocess.STDOUT, text=True,
        )

    def tearDown(self):
        if self.p.poll() is None:
            self.p.kill()
        self.p.wait(timeout=3)
        if self.p.stdin and not self.p.stdin.closed:
            self.p.stdin.close()
        self.salida.close()
        self.tmp.cleanup()

    def enviar(self, texto):
        self.p.stdin.write(texto + "\n")
        self.p.stdin.flush()

    def eventos(self):
        if not self.csv.exists():
            return []
        with self.csv.open() as f:
            return [r for r in csv.DictReader(f) if all(v is not None for v in r.values())]

    def esperar(self, nombre, cantidad=1, plazo=5):
        limite = time.monotonic() + plazo
        while time.monotonic() < limite:
            filas = [r for r in self.eventos() if r["evento"] == nombre]
            if len(filas) >= cantidad:
                return filas[-1]
            if self.p.poll() is not None:
                break
            time.sleep(0.01)
        self.salida.seek(0)
        self.fail(f"No aparece {nombre} ({cantidad}):\n{self.salida.read()}")

    def cerrar(self, eof=False):
        if eof:
            self.p.stdin.close()
        else:
            self.enviar("salir")
        self.assertEqual(self.p.wait(timeout=3), 0)
        self.assertEqual(self.eventos()[-1]["evento"], "cierre_completado")
        self.salida.seek(0)
        salida = self.salida.read()
        self.assertIn("4 hilos unidos", salida)
        self.assertIn("eventos perdidos=0", salida)
        self.comprobar_limpieza()

    def comprobar_limpieza(self):
        self.assertEqual(list(Path("/dev/mqueue").glob(f"rider_{self.p.pid}_*")), [])
        hijos = [r for r in self.eventos() if r["evento"] == "puente_iniciado"]
        if hijos:
            self.assertFalse(Path(f"/proc/{hijos[0]['id']}").exists(), "Hijo sin recoger")

    def test_movimientos_limitados_en_ambas_direcciones(self):
        self.enviar("avanzar 100")
        self.esperar("rechazada_enclavamiento")
        self.enviar("rearmar\navanzar 100\nretroceder 100")
        self.esperar("movimiento_completado", 2)
        inicios = [r for r in self.eventos() if r["evento"] == "movimiento_iniciado"]
        self.assertEqual([r["direccion"] for r in inicios], ["1", "-1"])
        fines = [r for r in self.eventos() if r["evento"] == "movimiento_completado"]
        for inicio, fin in zip(inicios, fines):
            self.assertGreaterEqual(int(fin["instante_puente_ms"]) - int(inicio["instante_puente_ms"]), 100)
            self.assertEqual(fin["direccion"], "0")
        self.cerrar()

    def test_parada_con_cola_llena_y_rearme_sin_reanudacion(self):
        self.enviar("rearmar\navanzar 3000")
        self.esperar("movimiento_iniciado")
        self.enviar("avanzar 100\n" * 12)
        self.esperar("rechazada_cola_llena")
        self.enviar("parar")
        parada = self.esperar("parada_solicitada")
        confirmacion = self.esperar("parada_confirmada")
        self.assertEqual(parada["pendientes"], "0")
        self.assertEqual(confirmacion["direccion"], "0")
        self.enviar("avanzar 100")
        self.esperar("rechazada_enclavamiento")
        self.enviar("rearmar\nestado")
        estado = self.esperar("estado_armado")
        self.assertEqual(estado["direccion"], "0")
        self.assertEqual(estado["pendientes"], "0")
        self.assertEqual(len([r for r in self.eventos() if r["evento"] == "movimiento_iniciado"]), 1)
        self.enviar("retroceder 80")
        self.esperar("movimiento_completado")
        inicios = [r for r in self.eventos() if r["evento"] == "movimiento_iniciado"]
        self.assertEqual(len(inicios), 2)
        self.assertNotEqual(inicios[0]["generacion"], inicios[1]["generacion"])
        self.cerrar()

    def test_orden_caducada_no_se_ejecuta(self):
        self.enviar("rearmar\navanzar 2500")
        self.esperar("movimiento_iniciado")
        self.enviar("retroceder 100")
        self.esperar("orden_descartada")
        self.assertEqual(len([r for r in self.eventos() if r["evento"] == "movimiento_iniciado"]), 1)
        self.cerrar()

    def test_entrada_invalida_y_eof(self):
        self.enviar("rearmar\navanzar 0\navanzar -1\navanzar 3001\navanzar 9999999999999999999999\navanzar 12x")
        self.esperar("duracion_invalida", 5)
        self.enviar("desconocido\nparar extra\navanzar 10 extra")
        self.esperar("comando_invalido", 3)
        self.enviar("x" * 200)
        self.esperar("linea_demasiado_larga")
        self.cerrar(eof=True)

    def test_eof_durante_movimiento(self):
        self.enviar("rearmar\navanzar 3000")
        self.esperar("movimiento_iniciado")
        self.cerrar(eof=True)
        self.assertIn("parada_confirmada", [r["evento"] for r in self.eventos()])

    def test_sigint_sin_entrada(self):
        self.enviar("estado")
        self.esperar("estado_enclavado")
        self.p.send_signal(signal.SIGINT)
        self.assertEqual(self.p.wait(timeout=3), 0)
        self.assertIn("senal_recibida", [r["evento"] for r in self.eventos()])
        self.assertEqual(self.eventos()[-1]["evento"], "cierre_completado")

    def test_sigterm_durante_movimiento(self):
        self.enviar("rearmar\navanzar 3000")
        self.esperar("movimiento_iniciado")
        self.p.send_signal(signal.SIGTERM)
        self.assertEqual(self.p.wait(timeout=3), 0)
        self.assertIn("parada_confirmada", [r["evento"] for r in self.eventos()])
        self.assertEqual(self.eventos()[-1]["evento"], "cierre_completado")

    def comprobar_fallo_y_recuperacion(self, modo, nombre):
        self.enviar("rearmar\navanzar 3000")
        self.esperar("movimiento_iniciado")
        self.enviar("retroceder 100\nsensor " + modo)
        fallo = self.esperar(nombre)
        confirmacion = self.esperar("parada_confirmada")
        self.assertEqual(fallo["pendientes"], "0")
        self.assertEqual(confirmacion["direccion"], "0")
        self.assertGreaterEqual(int(confirmacion["latencia_parada_us"]), 0)
        if modo == "invalido":
            self.assertEqual(fallo["valida"], "0")
        else:
            self.assertGreater(int(fallo["edad_telemetria_ms"]), 600)
        self.enviar("rearmar")
        self.esperar("rearme_rechazado_telemetria")
        self.enviar("sensor normal")
        self.esperar("telemetria_recuperada")
        self.enviar("avanzar 100\nestado")
        self.esperar("rechazada_enclavamiento")
        self.esperar("estado_enclavado")
        self.assertEqual(len([r for r in self.eventos() if r["evento"] == "movimiento_iniciado"]), 1)
        self.enviar("rearmar\nretroceder 80")
        self.esperar("movimiento_completado")
        inicios = [r for r in self.eventos() if r["evento"] == "movimiento_iniciado"]
        self.assertEqual(len(inicios), 2)
        self.assertEqual(inicios[-1]["direccion"], "-1")
        self.assertNotEqual(inicios[0]["generacion"], inicios[-1]["generacion"])
        self.cerrar()

    def test_telemetria_invalida_para_y_exige_rearme(self):
        self.comprobar_fallo_y_recuperacion("invalido", "fallo_telemetria_invalida")

    def test_telemetria_congelada_para_y_exige_rearme(self):
        self.comprobar_fallo_y_recuperacion("congelado", "fallo_telemetria_antigua")

    def test_temporizador_medidas_y_planificacion(self):
        self.esperar("ciclo_supervision", 3)
        self.enviar("carga 120")
        # La carga acumula expiraciones: se miden al consumir la senal siguiente.
        limite = time.monotonic() + 5
        while time.monotonic() < limite:
            ciclos = [r for r in self.eventos() if r["evento"] == "ciclo_supervision"]
            if any(int(r["expiraciones_perdidas"]) > 0 for r in ciclos):
                break
            time.sleep(0.01)
        else:
            self.fail("La carga no produjo expiraciones acumuladas")
        self.assertTrue(any(int(r["retraso_us"]) >= 100000 for r in ciclos))
        self.assertTrue(all(int(r["retraso_us"]) >= 0 for r in ciclos))
        self.assertTrue(all(int(r["cpu_hilo_us"]) >= 0 for r in ciclos))
        self.assertTrue(any(int(r["intervalo_us"]) > 0 for r in ciclos))
        planificacion = [r for r in self.eventos() if r["evento"].startswith("planificacion_")]
        self.assertEqual(len(planificacion), 4)
        self.assertTrue(all(int(r["politica"]) >= 0 for r in planificacion))
        self.assertTrue(all(int(r["prioridad"]) >= 0 for r in planificacion))
        self.cerrar()

    def test_caida_del_hijo_se_detecta_sin_confirmar_parada(self):
        self.enviar("rearmar\navanzar 3000")
        self.esperar("movimiento_iniciado")
        hijo = int(self.esperar("puente_iniciado")["id"])
        os.kill(hijo, signal.SIGKILL)
        self.assertNotEqual(self.p.wait(timeout=4), 0)
        self.assertIn("puente_caido", [r["evento"] for r in self.eventos()])
        self.assertNotIn("parada_confirmada", [r["evento"] for r in self.eventos()])
        self.comprobar_limpieza()

    def test_bloqueo_del_hijo_no_bloquea_el_cierre(self):
        self.enviar("rearmar\navanzar 3000")
        self.esperar("movimiento_iniciado")
        hijo = int(self.esperar("puente_iniciado")["id"])
        os.kill(hijo, signal.SIGSTOP)
        self.assertNotEqual(self.p.wait(timeout=4), 0)
        self.assertIn("puente_sin_respuesta", [r["evento"] for r in self.eventos()])
        self.assertNotIn("parada_confirmada", [r["evento"] for r in self.eventos()])
        self.comprobar_limpieza()

    def test_fallo_de_exec_limpia_las_colas(self):
        self.cerrar()
        p = subprocess.Popen([str(BINARIO), str(self.csv), "/no/existe/puente"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        _, error = p.communicate(timeout=4)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("inicio del puente", error)
        self.assertEqual(list(Path("/dev/mqueue").glob(f"rider_{p.pid}_*")), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
