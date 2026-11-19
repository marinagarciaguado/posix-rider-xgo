# posix-rider-xgo

Aplicación concurrente POSIX para el XGO Rider 2: avance y retroceso con duración
limitada, parada, rearme y supervisión. El alcance del proyecto es exclusivamente
el Rider. El desarrollo actual funciona en simulación; la integración física está
pendiente. El plan completo está en [PLAN.md](docs/PLAN.md).

## Estructura

```text
src/        Supervisor, puente simulado y transporte IPC en C.
include/    Cabeceras de IPC y protocolo compartido.
examples/   Ejemplo POSIX de referencia.
tests/      Pruebas de integración, protocolo y saturación.
docs/       Plan de implementación y especificación del protocolo.
build/      Ejecutables generados por make; excluidos de Git.
```

`README.md`, `Makefile` y `.gitignore` quedan en la raíz. Los comandos de
compilación se ejecutan desde esa carpeta.

El código, los comentarios, las pruebas y la interfaz de consola están en inglés.
Los comandos son `arm`, `forward MS`, `backward MS`, `stop`, `state`, `quit`,
`sensor normal|invalid|frozen` y `load MS`. Los eventos y las columnas del CSV
también usan nombres en inglés. Las convenciones están en [AGENTS.md](AGENTS.md).
Las validaciones se separan en funciones por responsabilidad, con comprobaciones
cortas y retornos tempranos.

## Paso 1 Ejemplo POSIX

`examples/posix_example.c` es la versión en inglés del ejemplo local de referencia con datos simulados; está pendiente cotejar su procedencia y correspondencia funcional con el archivo original del profesor. El hilo de sensores actualiza un contador cada 200 ms. El hilo de supervisión copia el contador bajo el mismo mutex y muestra una lectura cada 500 ms. Al final, el hilo principal espera a ambos con `pthread_join`.

Desde esta carpeta, en Linux o WSL con un compilador de C y `make` instalados:

```sh
make
./build/posix_example
```

Se esperan cinco líneas de supervisión y el mensaje `Done: both threads have finished.`. Los valores de las lecturas pueden variar según el orden en que se ejecuten los hilos.

`make run` compila si hace falta y ejecuta el ejemplo. `make clean` elimina el ejecutable.

## Paso 2: simulador del Rider

`src/rider_sim.c` implementa el supervisor de la aplicacion concurrente. Es una
simulacion sin conexion al hardware. El modelo ahora se ejecuta en el proceso
`rider_bridge_sim`, conservando `posix_example.c` como referencia.
Requiere Linux, compilador C11, `make` y, para las pruebas, Python 3.

Los cuatro hilos, la capacidad de la cola y los tiempos indicados abajo son
decisiones de implementación. No se han cotejado como requisitos del profesor
ni como límites del Rider físico. El enunciado original no está disponible en
el repositorio; las pruebas verifican el comportamiento del simulador.

```sh
make
make run-sim
```

Arranca parado y enclavado. Introduce cada orden y pulsa Intro:

```text
arm
forward 1000
backward 500
state
stop
arm
forward 200
quit
```

Las duraciones se expresan en milisegundos, de 1 a 3000. Los movimientos se
ejecutan en orden; las ordenes que esperen 2000 ms caducan. `stop` actua fuera
de la FIFO, vacia las pendientes e invalida su generacion. El hilo de intercambio
recibe del puente la confirmacion de parada simulada; si aun esta en movimiento
o falta esa confirmacion, `arm` se rechaza y
debe solicitarse otra vez tras `stop_confirmed`. Rearmar requiere despues
una orden nueva. `arm_requested` indica la peticion y `armed` la
confirmacion del puente. Se pueden encolar movimientos mientras se espera el
rearme, pero no se envian hasta recibir esa confirmacion.
`quit`, Ctrl+C, SIGTERM y fin de entrada cierran la aplicacion.
Si usas una tuberia, mantenla abierta para observar movimientos: EOF solicita
parada inmediata, aunque queden ordenes pendientes.

Los cuatro hilos son entrada, intercambio por IPC (10 ms), supervision (20 ms)
y registro. Comparten estado mediante mutex. La FIFO de ocho ordenes usa
semaforos POSIX para elementos y huecos, con reservas no bloqueantes coordinadas
bajo el mutex; una cola llena rechaza nuevas ordenes de movimiento. La
temporizacion usa `CLOCK_MONOTONIC`: esperas absolutas para el intercambio y
un temporizador POSIX para la supervision. Todos los hilos creados
se recogen con `pthread_join`, tambien si falla una creacion intermedia.

El registro escribe fuera del mutex y genera `rider_sim.csv`, con tiempo relativo
monotonico, evento, identificador de orden, generacion, pendientes y direccion
simulada (`1`, `-1` o `0`). Se puede elegir destino con
`./build/rider_sim /tmp/prueba.csv`; se sobrescribe ese archivo. El bufer de eventos es
acotado y el cierre informa de eventos perdidos si el consumidor no sigue el ritmo.

```sh
make test
```

Las pruebas comprueban movimiento en ambos sentidos, limites y entradas invalidas,
caducidad en cola, saturacion, parada, rearme sin reanudacion automatica y cierre
por EOF, SIGINT y SIGTERM. Usan registros temporales y plazos para detectar bloqueos.

## Paso 3: supervision y medidas en simulacion

La supervision usa `timer_create` y `timer_settime` con un periodo de 20 ms.
SIGRTMIN, SIGINT y SIGTERM se bloquean antes de crear los hilos. Solo el hilo de
supervision las consume con `sigtimedwait`, una espera sincrona con plazo de la
familia de `sigwait`. El plazo permite revisar el cierre aunque no llegue una
senal. El temporizador se elimina despues de unir los hilos.

El modelo publica una muestra inicial valida y la renueva cada 200 ms. La
supervision copia la muestra bajo mutex, la evalua fuera y comprueba que siga
siendo la misma antes de aplicar el resultado. Una muestra invalida o con mas
de 600 ms de antiguedad enclava el control y vacia las ordenes pendientes.
La recuperacion del sensor mantiene el enclavamiento: hay que pedir `arm`
y enviar una orden nueva.

Para comprobarlo de forma interactiva, envia cada orden por separado:

```text
arm
forward 3000
sensor invalid
```

Espera `invalid_telemetry_fault` y `stop_confirmed`. `arm` se rechaza
mientras persista el fallo. Envia `sensor normal`, espera
`telemetry_recovered` y solicita `arm`. `sensor frozen` deja de
publicar muestras y permite comprobar `stale_telemetry_fault`. El evento
`sensor_configured` confirma la configuracion, no la recuperacion de la muestra.

`load 120` introduce una unica carga activa de 120 ms en el hilo de supervision,
fuera del mutex. Acepta de 1 a 200 ms; otra orden sustituye una carga pendiente.
Permite observar retrasos y expiraciones acumuladas mientras el intercambio
sigue trabajando. Estas ordenes de inyeccion pertenecen solo al simulador.

El CSV conserva las seis primeras columnas y añade estas medidas:

| Columnas | Interpretacion |
| --- | --- |
| `sample`, `sample_age_ms`, `valid` | Secuencia, edad al registrar el evento y validez de la ultima muestra. |
| `lateness_us` | Retraso desde la activacion nominal hasta iniciar las comprobaciones del ciclo; incluye la carga inyectada. |
| `missed_expirations` | Expiraciones adicionales acumuladas en la notificacion consumida, obtenidas con `timer_getoverrun`. No es un contador acumulativo de toda la ejecucion. |
| `interval_us`, `jitter_us` | Tiempo entre comprobaciones y diferencia respecto al periodo multiplicado por las expiraciones representadas por la notificacion actual. |
| `stop_latency_us` | Tiempo desde la peticion de parada durante movimiento hasta recibir su confirmacion del puente; incluye el intercambio IPC. |
| `policy`, `priority` | Valores efectivos consultados con `pthread_getschedparam` al iniciar cada uno de los cuatro hilos, en eventos `scheduling_*`. No se cambia la politica heredada. |
| `thread_cpu_us` | Tiempo de CPU acumulado del hilo de supervision, obtenido con `CLOCK_THREAD_CPUTIME_ID`. |
| `bridge_timestamp_ms` | Instante monotonico absoluto del evento en el puente, en respuestas registradas. Permite medir la duracion del modelo sin confundirla con el instante de recepcion (`ms`, relativo al inicio del supervisor). |

Las medidas de ciclo solo aparecen en `supervision_cycle`; la latencia de parada,
en `stop_confirmed`; politica y prioridad, en `scheduling_*`. Un valor `-1`
indica que una medida no aplica o no esta disponible; el jitter del primer ciclo
es cero. Los eventos periodicos quedan en CSV para no saturar la consola.
Al salir se muestran ciclos, total de expiraciones perdidas, ciclos con un retraso
de al menos 20 ms (`late_cycles`), retraso maximo y latencia maxima de parada.

Las pruebas de supervision incluyen telemetria invalida, telemetria congelada
y temporizador con carga y medidas de planificacion.
Las pruebas verifican comportamiento y coherencia, sin imponer una latencia
maxima del sistema operativo. La planificacion en tiempo real sigue pendiente
de evaluar en el entorno de destino.

Quedan pendientes la integracion con el SDK y las pruebas fisicas del
Rider, segun [PLAN.md](docs/PLAN.md). Los periodos y umbrales anteriores son decisiones
del simulador; no representan garantias de tiempo real ni tiempos de parada
del robot fisico.

Referencias tecnicas: [temporizadores POSIX](https://man7.org/linux/man-pages/man2/timer_create.2.html),
[espera sincrona de senales](https://man7.org/linux/man-pages/man2/sigwaitinfo.2.html)
y [expiraciones acumuladas](https://man7.org/linux/man-pages/man2/timer_getoverrun.2.html).

## Paso 4: puente simulado entre procesos

`make run-sim` compila ambos programas y ejecuta el supervisor. Antes de crear
los hilos, este crea dos colas POSIX con nombres exclusivos y permisos `0600`,
hace `fork` y ejecuta `rider_bridge_sim` con `exec`. Espera su mensaje de listo
antes de admitir ordenes. El puente se busca junto al ejecutable del supervisor,
tambien al lanzarlo desde otra carpeta; en Linux se resuelve con `/proc/self/exe`.
Se puede indicar otra ruta con `./build/rider_sim /tmp/prueba.csv /path/rider_bridge_sim`.

El hilo de intercambio es el unico productor de ordenes IPC. El puente es el
unico propietario del modelo de movimiento y de las muestras simuladas. La
direccion que muestra el supervisor cambia por respuestas del puente. Al
perder el hijo no se inventa una confirmacion de parada: el registro conserva
el ultimo estado conocido y la aplicacion termina con error.

Cada cola admite diez mensajes de 96 bytes. El trafico normal de ordenes se
limita a seis mensajes ocupados para reservar capacidad para control; las
respuestas periodicas se limitan a ocho. Parada, rearme y cierre tienen mayor
prioridad que los movimientos. El protocolo comprueba version, longitud,
secuencias, generacion y caducidad. La parada invalida mensajes anteriores
incluso si siguen en transito. El formato esta descrito en [PROTOCOL.md](docs/PROTOCOL.md).
El puente actual esta escrito en C y se prueba tambien con un emisor Python
que codifica el formato de manera independiente.

El supervisor envia latidos cada 100 ms. Si su permiso de actividad de 500 ms
vence, el puente operativo para el modelo y se desarma. Tambien limita por si
mismo la duracion del movimiento. El supervisor detecta la salida del hijo con
`waitpid(..., WNOHANG)` y la ausencia de respuestas tras 600 ms. Estos fallos
enclavan el control y cierran la aplicacion; no se reinicia el puente automaticamente.

El cierre solicita parada, espera confirmacion y solicita la salida del hijo.
Si el protocolo no responde, se abandona la espera tras 500 ms y se recoge el
hijo con intentos acotados y senales de terminacion, incluido un hijo detenido.
El supervisor cierra y desenlaza sus colas tambien ante error de `exec`, caida o
bloqueo del hijo. La terminacion forzada se comunica como error. Esto verifica
el ciclo de vida del simulador; no acredita una parada fisica si un futuro
puente de hardware falla.

`make test` ejecuta una prueba C de reserva de capacidad y prioridad, mas
dieciocho pruebas Python. Cubren las funciones anteriores, caducidad en transito,
generaciones y secuencias antiguas, version y tamano invalidos, watchdog, fallo
de `exec`, muerte y bloqueo del hijo. Comprueban tambien la recogida del proceso
y la ausencia de colas residuales en los cierres probados. Las pruebas directas
del protocolo usan `ctypes` y `librt.so.1`, sin paquetes Python adicionales.

El entorno Linux debe admitir colas de mensajes POSIX de diez mensajes; no se
cambian limites del kernel. Referencia: [colas POSIX en Linux](https://man7.org/linux/man-pages/man7/mq_overview.7.html).
