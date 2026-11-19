# Plan de implementación del proyecto POSIX del XGO Rider 2

El proyecto se centra exclusivamente en el XGO Rider 2. El operador podrá avanzar y retroceder durante un tiempo limitado, parar y rearmar el control. El Rider conservará sus controladores de fábrica. La aplicación concurrente se desarrollará primero en simulación y después se conectará al robot mediante su SDK.

## Alcance del proyecto

El alcance confirmado por el usuario es exclusivamente el **Rider**. La propuesta técnica incluye `pthread_create` y `pthread_join`, `fork`/`exec`/`waitpid`, mutex, semáforos POSIX, colas de mensajes POSIX, temporizador POSIX y el patrón productor-consumidor para las órdenes y el registro. También propone documentar la política efectiva de planificación, empezando con `SCHED_OTHER` y evaluando `SCHED_FIFO` o `SCHED_RR` si procede. Esta lista es el plan técnico actual; su obligatoriedad debe cotejarse con el enunciado original.

### Procedencia y límites de la revisión

Revisión del 19 de noviembre de 2026: el repositorio no contiene el enunciado ni las instrucciones originales del profesor. Los enlaces de ChatGPT Library conservados al final no han permitido leer esos documentos en esta revisión. Por tanto, no se da por verificado el cumplimiento del enunciado ni se atribuyen al profesor los detalles de esta arquitectura.

Los cuatro hilos, la FIFO de ocho órdenes, el límite de 3000 ms, la caducidad de 2000 ms, los períodos de intercambio de 10 ms, supervisión de 20 ms y muestreo de 200 ms, el umbral de antigüedad de 600 ms y el CSV son decisiones de implementación del simulador. La arquitectura del puente simulado también es una decisión técnica. No son especificaciones verificadas del robot ni requisitos confirmados del profesor.

`examples/posix_example.c` es la versión en inglés del ejemplo local de referencia. Su procedencia y correspondencia funcional con el archivo original enlazado están pendientes de cotejo; no es una copia intacta del material del profesor.

La supervisión ya usa `timer_create`, `timer_settime` y `sigtimedwait` (espera síncrona con plazo de la familia de `sigwait`). El puente simulado entre procesos también está implementado; el siguiente bloque es identificar e integrar el SDK instalado en el Rider. `posix_example.c` contiene dos hilos con datos simulados y períodos de 200 y 500 ms. Se registrará por separado la demo del fabricante usada como punto de partida del Rider.

## Decisiones para el Rider

| Aspecto | Decisión y comprobación |
| --- | --- |
| Entrada | Consola local para `forward`, `backward`, `stop`, `arm` y `quit`. Las órdenes de movimiento llevan duración acotada y caducidad. |
| Hilos C | Entrada de órdenes, intercambio simulado, supervisión periódica y registro. La supervisión copia la telemetría bajo mutex, evalúa la copia y revalida su secuencia antes de aplicar el resultado. El registro consume eventos de un búfer acotado y escribe CSV fuera de la ruta crítica. |
| Cola interna | FIFO de ocho órdenes con mutex y semáforos para huecos y elementos. Las órdenes normales se rechazan cuando la cola está llena. Usaremos operaciones que no esperan mientras se tiene el mutex, con el vaciado coordinado para conservar los contadores. |
| Parada y rearme | `stop` y los fallos activan un enclavamiento fuera de la FIFO. Se vacía la FIFO y se invalida la generación anterior. El puente descarta mensajes antiguos y confirma la parada del modelo; el SDK sigue pendiente. El rearme requiere parada confirmada por el puente, muestra sana, modo de sensor normal y petición explícita. El movimiento solo se despacha tras confirmar el rearme. |
| IPC | Antes de crear hilos se inicia el puente con `fork` y `exec`; se recoge con `waitpid`. Dos colas POSIX llevan órdenes y respuestas. El despachador C es el único productor de órdenes y reserva capacidad para control prioritario. Se validan versión, tamaño, secuencias, generación y caducidad en un formato de 96 bytes compartido por dos programas C y probado con un emisor Python independiente. |
| Hardware | Se propone que el puente sea el único propietario del SDK y de su interfaz de comunicación. Antes de programar llamadas físicas se verificarán la API instalada, las unidades, el tiempo de las llamadas y el mecanismo de parada del Rider. No hay llamadas al SDK en el código actual. |
| Temporización | Se usa `CLOCK_MONOTONIC`: intercambio con espera absoluta a 10 ms y supervisión mediante temporizador POSIX a 20 ms. El puente publica muestras simuladas cada 200 ms y estado cada 100 ms. El registro escribe y vacía el archivo por evento. Se miden retrasos, intervalos, jitter, expiraciones acumuladas, CPU del supervisor y latencia de parada, que ahora incluye el IPC de ida y confirmación. Los eventos de movimiento incluyen el instante del puente para separar duración y recepción. |

Un período de 20 ms indica cada cuánto pedimos ejecutar la supervisión. Con el muestreo simulado de 200 ms se inspecciona la última lectura disponible. En hardware un cambio físico podrá ocurrir entre muestras. La edad de la muestra y la latencia desde la petición hasta recibir la confirmación del puente se registran por separado. Quedan pendientes las medidas con el SDK y de detención física. Los períodos configurados no garantizan tiempos máximos de respuesta.

El puente simulado limita la duración de la orden activa y vigila la caducidad de los mensajes y de los latidos del supervisor. Si el puente muere o se bloquea, el supervisor detecta el problema, enclava y cierra con error sin afirmar que hubo parada confirmada. La respuesta física con un futuro puente de hardware caído dependerá del firmware y del método físico de intervención validado en el laboratorio. Enviar una orden por el SDK tampoco confirma por sí solo que el robot se haya detenido. Las pruebas deliberadas de bloqueo y terminación del puente ya se realizan exclusivamente en simulación.

La primera versión contempla avance, retroceso, parada, rearme y consulta de estado. Cualquier condición de parada basada en sensores requerirá verificar primero las lecturas disponibles y justificar sus umbrales con el equipo físico.

## Fases de implementación y criterios para pasar a la siguiente

| Fase | Trabajo | Resultado comprobable |
| --- | --- | --- |
| 0. Inventario y fuentes originales | Conservar y ejecutar `posix_example.c` como referencia local. Obtener y cotejar el enunciado, las instrucciones y el ejemplo originales. Identificar y registrar imagen, kernel, SDK, demo oficial, mecanismo de parada y comportamiento observable del Rider. | Requisitos con referencia a su fuente y ficha del Rider con versiones, código de partida y pasos para reproducir la demo. Podemos avanzar en simulación mientras se completa el cotejo y el inventario físico. |
| 1. Rider simulado en C | Crear `Makefile` y el ejecutable con temporización monotónica, estado compartido y cuatro hilos. Implementar la consola, FIFO, simulación del adaptador, caducidad, parada prioritaria, rearme y cierre por `Ctrl+C`. | Compila en Linux. Avanza y retrocede con tiempos limitados en simulación, para con cola llena y termina con `pthread_join`. |
| 2. Supervisión y medidas | Incorporar el temporizador POSIX, vigilar edad y validez de la telemetría, registrar latencias, retrasos y plazos superados. Inyectar lecturas inválidas o caducadas y carga controlada. | CSV reproducible, parada ante fallo y recuperación sin reanudación automática. Se registra política y prioridad efectivas. |
| 3. Puente simulado | Crear el protocolo y las colas POSIX, iniciar el puente simulado desde C y validar el intercambio. Probar saturación, órdenes caducadas en tránsito, caída o bloqueo del hijo, cierre y limpieza de recursos. | C y el puente explican la misma secuencia de eventos. El canal mantiene capacidad para control bajo saturación, invalida órdenes antiguas y limpia colas y proceso hijo. |
| 4. Rider físico | Sustituir el adaptador simulado por el SDK instalado. Verificar inicialmente las lecturas, la vía de parada y la reacción ante pérdida de comunicación siguiendo el procedimiento de laboratorio. Probar después secuencias breves supervisadas. | Vídeo o registro de avance, retroceso y parada en el Rider real, con las limitaciones observadas. |
| 5. Validación y entrega | Repetir en el Rider la demo de fábrica y la versión concurrente con acciones comparables, interferencia controlada y fallo o recuperación. Registrar condiciones, repeticiones, jitter, latencias, retrasos, CPU y política efectiva. Preparar instrucciones, memoria y vídeo. | Repositorio reproducible con código del Rider, procedencia del código base y resultados del dispositivo. |

## Primer paso de código

Conservar `posix_example.c` como referencia local, comprobar la compilación y la salida en Linux y montar el simulador del Rider sin depender todavía del acceso al robot. Estos pasos de código ya están realizados; el cotejo con las fuentes originales, el inventario y las pruebas físicas del Rider siguen pendientes.

## Avance de codigo: simulador del Rider

Se ha implementado `src/rider_sim.c`, con cuatro hilos, FIFO de ocho ordenes,
mutex y semaforos POSIX, duraciones limitadas, caducidad, parada fuera de la
FIFO, generaciones, rearme explicito y cierre por consola, EOF o senal.
El registro CSV se escribe desde un bufer acotado fuera de la ruta de control.
`make test` ejecuta las pruebas de integracion de esta fase en Linux.
La fase 2 de simulacion ya incorpora temporizador POSIX, telemetria valida,
invalida o congelada y carga controlada fuera del mutex. Un fallo enclava y vacia
la cola; recuperar el sensor no rearma ni reanuda movimientos automaticamente.
Se registran retrasos, jitter, expiraciones perdidas, latencia de parada,
politica y prioridad efectivas de los cuatro hilos y CPU del supervisor.
Las señales de terminacion se consumen de forma sincrona y el temporizador
se elimina tras unir los hilos. Las pruebas con politicas de tiempo real
siguen pendientes de evaluar en el entorno de destino.

La fase 3 incorpora `src/rider_bridge_sim.c`, `src/rider_ipc.c` y el protocolo de
`include/rider_protocol.h`. El puente es un proceso C independiente creado antes de los
hilos. Solo él modifica el modelo de movimiento y publica telemetría.
Hay dos colas de diez mensajes, reserva para control, prioridad de parada,
latidos y watchdog. El cierre recoge el hijo y limpia las colas; las pruebas
cubren también ausencia de confirmación ante caída o bloqueo del puente.
El formato y la semántica están documentados en [PROTOCOL.md](PROTOCOL.md).
El código está separado en `src/`, las cabeceras en `include/`, el ejemplo en
`examples/`, las pruebas en `tests/` y la documentación en `docs/`. `make`
genera los ejecutables en `build/`, excluida del control de versiones.

Validacion realizada el 19 de noviembre de 2026 en un contenedor Linux con
la imagen oficial `gcc:14-bookworm`: el ejemplo, el supervisor y el puente compilan con
`-std=c11 -Wall -Wextra -Wpedantic -Werror -O2`, el ejemplo docente termina
con sus cinco lecturas en la validacion inicial. Tras incorporar la fase 3,
se valida la compilación con los mismos indicadores, una prueba C de capacidad
y prioridad de las colas y dieciocho pruebas Python de integración y protocolo.
El repositorio se monto en solo lectura y la compilacion se hizo en una copia
temporal dentro del contenedor. Esta validacion no sustituye las pruebas fisicas.

## Referencias conservadas para cotejo

Estos enlaces no acreditan por sí solos que su contenido se haya verificado en esta revisión.

- [Enunciado del profesor](https://chatgpt.com/library?file_id=libfile_e2316518fe2c81918e9b8be97553cbdf&account_id=personal)
- [Instrucciones del Rider](https://chatgpt.com/library?file_id=libfile_f3bd072691b481919d4f6faf661f4709&account_id=personal)
- [Especificación de arquitectura](https://chatgpt.com/library?file_id=libfile_90132a1e000c81919cb3ad36a85fe5b7&account_id=personal) y [explicación paso a paso](https://chatgpt.com/library?file_id=libfile_b015de1505e08191bd41e67fb024a40d&account_id=personal)
- [Ejemplo docente posix_example.c](https://chatgpt.com/library?file_id=libfile_efa012244ad88191b4cff59fbd7490e3&account_id=personal)
- [Documentación oficial del XGO Rider 2](https://wiki.xgorobot.com/kb/the-xgo-series-robot/models/rider2)
- [Manual de Linux para mq_send y mq_timedsend](https://www.man7.org/linux/man-pages/man2/mq_timedsend.2.html)
