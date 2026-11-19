#define _POSIX_C_SOURCE 200809L
#include "rider_ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Unico propietario del modelo. Sin SDK ni acceso al robot. */
typedef struct {
    mqd_t ordenes, respuestas;
    pid_t padre;
    uint64_t secuencia, generacion, orden, muestra, recibidas[6], secuencia_parada;
    int direccion, sensor;
    bool armado, valida, error_canal, watchdog;
    int64_t inicio_ms, fin_ms, muestra_ms, proxima_muestra_ms, lease_ms, estado_ms;
} Modelo;

static int64_t ms(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) _exit(2);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}
static int responder(Modelo *b, uint32_t tipo, uint32_t motivo, uint64_t orden) {
    int64_t ahora = ms();
    Mensaje m = {
        .tipo = tipo, .flags = (b->armado ? RIDER_ARMADO : 0) |
                             (b->valida ? RIDER_MUESTRA_VALIDA : 0),
        .secuencia = b->secuencia + 1, .generacion = b->generacion,
        .orden_id = orden, .caduca_ms = ahora + RIDER_TIMEOUT_MS,
        .direccion = b->direccion, .sensor = (uint32_t)b->sensor, .motivo = motivo,
        .muestra = b->muestra, .muestra_ms = b->muestra_ms,
        .instante_ms = tipo == R_INICIO ? b->inicio_ms : ahora, .fin_ms = b->fin_ms
    };
    bool normal = tipo == R_ESTADO;
    int rc = rider_enviar(b->respuestas, &m, normal ? 0 : 20,
                         normal ? RIDER_RESP_NORMAL_MAX : RIDER_MQ_MAX);
    if (rc == 1) ++b->secuencia;
    if (rc < 0 || (rc == 0 && !normal)) {
        b->error_canal = true;
        b->armado = false;
        b->direccion = 0;
    }
    return rc;
}
static void detener(Modelo *b) {
    b->armado = false;
    b->direccion = 0;
    b->orden = 0;
    b->fin_ms = 0;
}
static void descartar(Modelo *b, const Mensaje *m, uint32_t motivo) {
    responder(b, R_DESCARTE, motivo, m->orden_id);
}
static bool procesar(Modelo *b, const Mensaje *m) {
    int64_t ahora = ms();
    if (m->tipo < C_LATIDO || m->tipo > C_SALIR) {
        detener(b);
        responder(b, R_FALLO, MOTIVO_PROTOCOLO, 0);
        return false;
    }
    /* Cada clase tiene su secuencia: las prioridades pueden adelantar un
       control a un movimiento sin convertir este ultimo en un replay. */
    if (m->secuencia <= b->recibidas[m->tipo]) {
        descartar(b, m, MOTIVO_PROTOCOLO);
        return false;
    }
    b->recibidas[m->tipo] = m->secuencia;
    if (m->caduca_ms <= ahora || m->instante_ms > ahora) {
        descartar(b, m, MOTIVO_CADUCADO);
        return false;
    }
    if (m->generacion < b->generacion ||
        (m->generacion != b->generacion && m->tipo != C_PARAR && m->tipo != C_SALIR)) {
        descartar(b, m, MOTIVO_GENERACION);
        return false;
    }
    if ((m->tipo == C_REARMAR || m->tipo == C_MOVER) && m->secuencia <= b->secuencia_parada) {
        descartar(b, m, MOTIVO_PROTOCOLO);
        return false;
    }
    switch (m->tipo) {
    case C_PARAR:
        b->generacion = m->generacion;
        b->secuencia_parada = m->secuencia;
        detener(b);
        responder(b, R_PARADA, MOTIVO_NINGUNO, 0);
        break;
    case C_SALIR:
        b->generacion = m->generacion;
        detener(b);
        responder(b, R_SALIDA, MOTIVO_NINGUNO, 0);
        return true;
    case C_LATIDO:
        if (m->caduca_ms > ahora + RIDER_LEASE_MS) {
            descartar(b, m, MOTIVO_PROTOCOLO);
            break;
        }
        b->lease_ms = m->caduca_ms;
        b->watchdog = false;
        b->sensor = (int)m->sensor;
        break;
    case C_REARMAR:
        if (b->direccion != 0 || b->error_canal || b->watchdog || ahora >= b->lease_ms ||
            b->sensor != 0 || !b->valida || ahora - b->muestra_ms > RIDER_EDAD_MAX_MS) {
            descartar(b, m, MOTIVO_SENSOR);
            break;
        }
        b->armado = true;
        responder(b, R_REARME, MOTIVO_NINGUNO, 0);
        break;
    case C_MOVER:
        if (!b->armado || b->watchdog || ahora >= b->lease_ms) descartar(b, m, MOTIVO_NO_ARMADO);
        else if (b->direccion != 0) descartar(b, m, MOTIVO_OCUPADO);
        else if (!b->valida || ahora - b->muestra_ms > RIDER_EDAD_MAX_MS) descartar(b, m, MOTIVO_SENSOR);
        else if (m->orden_id == 0 || m->duracion_ms < 1 || m->duracion_ms > RIDER_MAX_MS ||
                 (m->direccion != 1 && m->direccion != -1)) descartar(b, m, MOTIVO_PROTOCOLO);
        else {
            b->direccion = m->direccion;
            b->orden = m->orden_id;
            b->inicio_ms = ahora;
            b->fin_ms = ahora + m->duracion_ms;
            responder(b, R_INICIO, MOTIVO_NINGUNO, b->orden);
        }
        break;
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc != 3) { fprintf(stderr, "Uso: %s COLA_ORDENES COLA_RESPUESTAS\n", argv[0]); return 2; }
    /* Ctrl+C corresponde al supervisor; SIGTERM queda disponible para recoger
       un hijo que no responde al protocolo de cierre. */
    struct sigaction ignorar = {.sa_handler = SIG_IGN};
    sigemptyset(&ignorar.sa_mask);
    sigset_t vacia;
    sigemptyset(&vacia);
    if (sigaction(SIGINT, &ignorar, NULL) != 0 || sigprocmask(SIG_SETMASK, &vacia, NULL) != 0)
        return 2;
    Modelo b = {
        .ordenes = mq_open(argv[1], O_RDONLY | O_NONBLOCK),
        .respuestas = mq_open(argv[2], O_WRONLY | O_NONBLOCK),
        .padre = getppid(), .generacion = 1, .muestra = 1, .valida = true
    };
    if (b.ordenes == (mqd_t)-1 || b.respuestas == (mqd_t)-1) {
        perror("mq_open puente");
        if (b.ordenes != (mqd_t)-1) mq_close(b.ordenes);
        if (b.respuestas != (mqd_t)-1) mq_close(b.respuestas);
        return 2;
    }
    b.muestra_ms = ms();
    b.proxima_muestra_ms = b.muestra_ms + RIDER_MUESTREO_MS;
    b.lease_ms = b.muestra_ms + RIDER_LEASE_MS;
    responder(&b, R_LISTO, MOTIVO_NINGUNO, 0);
    bool salir = false;
    int resultado = 0;
    while (!salir) {
        int64_t ahora = ms();
        if (getppid() != b.padre) { detener(&b); resultado = 2; break; }
        if (ahora >= b.lease_ms && !b.watchdog) {
            b.watchdog = true;
            detener(&b);
            responder(&b, R_FALLO, MOTIVO_WATCHDOG, 0);
        }
        if (b.error_canal) {
            detener(&b);
            /* Se reintenta un unico aviso de fallo sin bloquear el actuador. */
            if (responder(&b, R_FALLO, MOTIVO_SATURACION, 0) == 1) b.error_canal = false;
        }
        if (ahora >= b.proxima_muestra_ms) {
            if (b.sensor != 2) {
                ++b.muestra;
                b.muestra_ms = ahora;
                b.valida = b.sensor == 0;
            }
            b.proxima_muestra_ms = ahora + RIDER_MUESTREO_MS;
        }
        if (b.direccion != 0 && ahora >= b.fin_ms) {
            uint64_t id = b.orden;
            b.direccion = 0;
            b.orden = 0;
            responder(&b, R_FIN, MOTIVO_NINGUNO, id);
        }
        for (int i = 0; i < RIDER_MQ_MAX && !salir; ++i) {
            Mensaje m;
            unsigned prioridad;
            int rc = rider_recibir(b.ordenes, &m, &prioridad);
            if (rc == 0) break;
            if (rc < 0) {
                detener(&b);
                responder(&b, R_FALLO, MOTIVO_PROTOCOLO, 0);
                if (rc == -1) { resultado = 2; salir = true; }
                break;
            }
            salir = procesar(&b, &m);
        }
        if (!salir && ahora >= b.estado_ms) {
            responder(&b, R_ESTADO, MOTIVO_NINGUNO, b.orden);
            b.estado_ms = ahora + 100;
        }
        if (!salir) {
            struct timespec t = {.tv_nsec = 10000000};
            while (nanosleep(&t, &t) != 0 && errno == EINTR) { }
        }
    }
    detener(&b);
    mq_close(b.ordenes);
    mq_close(b.respuestas);
    return resultado;
}
