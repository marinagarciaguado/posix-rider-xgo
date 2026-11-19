# Protocolo del puente simulado del Rider, versión 1

Es una decisión de implementación del proyecto, pendiente de contrastar con el
enunciado. No es un protocolo del fabricante ni una interfaz de hardware.
Actualmente los dos procesos están escritos en C. Las pruebas de protocolo
codifican y decodifican los mensajes también desde Python de forma independiente.

## Transporte y formato

Dos colas POSIX no bloqueantes, una por sentido, con diez mensajes de 96 bytes.
El supervisor crea nombres exclusivos con `O_EXCL` y permisos `0600`; únicamente
desenlaza las colas que consiguió crear. Se pasan los nombres al hijo mediante
argumentos de `exec`. Los descriptores de cola se vuelven a abrir en el puente.

Los enteros son sin signo y big-endian, sin padding. Los tiempos monotónicos son
milisegundos absolutos del mismo equipo Linux y deben caber en un entero con
signo de 64 bits. El formato no sirve para relojes de equipos diferentes.

| Offset | Bytes | Campo |
| --- | --- | --- |
| 0 | 4 | Magic `0x52445231` (`RDR1`). |
| 4 | 2 | Versión: 1. |
| 6 | 2 | Longitud: 96. Se exige también que el mensaje recibido mida 96. |
| 8 | 4 | Tipo. |
| 12 | 4 | Flags: bit 0 armado, bit 1 muestra válida. |
| 16 | 8 | Secuencia no nula, creciente en cada emisor. |
| 24 | 8 | Generación no nula del control. |
| 32 | 8 | Identificador de movimiento, o cero. |
| 40 | 8 | Caducidad absoluta del mensaje. |
| 48 | 4 | Duración solicitada del movimiento, en ms; cero en otros mensajes. |
| 52 | 4 | Dirección codificada: 0 retroceso, 1 parada, 2 avance. |
| 56 | 4 | Sensor simulado: 0 normal, 1 inválido, 2 congelado. |
| 60 | 4 | Motivo de respuesta. |
| 64 | 8 | Secuencia de muestra; cero en órdenes. |
| 72 | 8 | Instante de muestra; cero en órdenes. |
| 80 | 8 | Instante de emisión o de transición al iniciar un movimiento. |
| 88 | 8 | Fin previsto del movimiento en el puente; cero si no aplica. |

## Mensajes y prioridades

| Tipo | Código | Prioridad |
| --- | --- | --- |
| Latido y configuración de sensor | 1 | 10 |
| Parada | 2 | 30 |
| Rearme | 3 | 20 |
| Movimiento | 4 | 0 |
| Salida | 5 | 31 |
| Listo, estado, inicio, fin, parada, rearme, descarte, fallo, salida | 16..24, respectivamente | Estado: 0; los demás: 20 |

Los movimientos y latidos solo se envían si la ocupación es menor de seis;
el control puede ocupar las diez plazas. Los estados periódicos solo se envían
si hay menos de ocho respuestas pendientes. Si no cabe una respuesta de control,
el puente detiene el modelo, se desarma y reintenta un aviso de fallo.
No se garantiza movimiento continuo si el consumidor deja de leer.

La prioridad puede alterar el orden de entrega. El puente controla secuencias
por tipo y aplica además la secuencia de parada como barrera para movimientos y
rearmes anteriores. El supervisor controla por separado las secuencias de
respuestas periódicas y de control. Las respuestas antiguas no sustituyen una
muestra con secuencia más reciente. Se rechazan generaciones antiguas; solo
parada y salida pueden adelantar la generación del puente.

Motivos: 0 ninguno, 1 caducado, 2 generación, 3 no armado, 4 ocupado,
5 sensor, 6 watchdog, 7 saturación, 8 protocolo.

## Estado y ciclo de vida

El puente comienza parado y desarmado. El supervisor espera `listo` antes de
crear sus cuatro hilos. Un rearme requiere una muestra sana, sensor normal y
latido vigente. La FIFO puede aceptar órdenes durante el rearme solicitado,
pero solo se envía un movimiento cuando el puente confirma que está armado.
Solo se mantiene un movimiento en tránsito o activo; el resto espera en la FIFO.

La parada del supervisor incrementa la generación, vacía la FIFO y bloquea
movimientos. Su confirmación exige una respuesta de parada de esa generación
con dirección cero y flag armado apagado. Recuperar telemetría no rearma.

Los latidos se solicitan cada 100 ms con validez de 500 ms. Si vence ese permiso,
el puente operativo detiene el modelo y avisa de fallo. También vigila por sí
mismo la duración activa. Una caída o bloqueo del puente no puede confirmar
una parada: el supervisor conserva el último estado conocido, cierra con error
y recoge el hijo. No se reinicia automáticamente.

El cierre normal solicita y confirma parada, solicita salida y recoge al hijo
con `waitpid`. Si no hay respuesta, se abandona la espera de cierre tras 500 ms;
la recogida intenta esperar 250 ms, envía SIGCONT y SIGTERM, espera otros 250 ms
y, si hace falta, usa SIGKILL y `waitpid`. Luego cierra y desenlaza ambas colas.
No se garantiza limpieza si el propio supervisor es terminado con SIGKILL.

Referencia del transporte: [manual de colas POSIX de Linux](https://man7.org/linux/man-pages/man7/mq_overview.7.html).
