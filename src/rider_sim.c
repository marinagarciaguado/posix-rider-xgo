#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "rider_ipc.h"

/* Simulador exclusivamente: ninguna funcion comunica con hardware. */
/* Parametros elegidos para esta implementacion; no son limites del robot
   ni requisitos cotejados con el enunciado original del profesor. */
enum {
    CAPACIDAD = 8, EVENTOS = 256, MAX_MS = RIDER_MAX_MS, CADUCIDAD_MS = 2000,
    SUPERVISION_MS = 20, EDAD_MAX_MS = RIDER_EDAD_MAX_MS, CARGA_MAX_MS = 200
};
enum { SENSOR_NORMAL, SENSOR_INVALIDO, SENSOR_CONGELADO };
enum { TELEMETRIA_SANA, TELEMETRIA_INVALIDA, TELEMETRIA_ANTIGUA };
typedef struct {
    uint64_t secuencia;
    int64_t ms;
    bool valida;
} Telemetria;
typedef struct {
    uint64_t id, generacion;
    int direccion, duracion_ms;
    int64_t caduca;
} Orden;
typedef struct {
    int64_t ms;
    uint64_t id, generacion;
    size_t pendientes;
    int direccion;
    uint64_t muestra;
    int64_t edad_telemetria_ms, retraso_us, intervalo_us, jitter_us;
    int64_t latencia_parada_us, cpu_hilo_us, instante_puente_ms;
    int valida, sobrepasos, politica, prioridad;
    char nombre[48];
} Evento;
typedef struct {
    /* Todas las reservas de semaforo son trywait bajo este mutex.
       Nadie reserva elementos/huecos fuera: vaciar no deja reservas en vuelo. */
    pthread_mutex_t mutex;
    pthread_cond_t evento_disponible;
    sem_t huecos, elementos;
    Orden cola[CAPACIDAD];
    size_t cabeza, cantidad;
    uint64_t siguiente_id, generacion;
    bool enclavado, cerrando, registro_finalizado;
    int direccion, error;
    uint64_t activa_id;
    int64_t fin_movimiento, inicio;
    Telemetria telemetria;
    int sensor, fallo_telemetria, carga_ms;
    int64_t solicitud_parada_ns;
    Puente puente;
    bool puente_armado, parada_confirmada, orden_enviada, salida_enviada;
    uint64_t ipc_secuencia, recibidas[2], parada_enviada, rearme_enviado;
    int64_t ultima_respuesta_ms, ultimo_latido_ms, control_ms;
    timer_t temporizador;
    sigset_t senales;
    int64_t siguiente_supervision_ns;
    uint64_t ciclos, expiraciones_perdidas, plazos_superados;
    int64_t retraso_max_us, latencia_parada_max_us;
    Evento eventos[EVENTOS];
    size_t evento_cabeza, evento_cantidad;
    unsigned long eventos_perdidos;
    FILE *csv;
} Aplicacion;

static int64_t ahora_ns(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
        perror("clock_gettime");
        abort();
    }
    return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

static int64_t ahora_ms(void) {
    return ahora_ns() / 1000000;
}

static int salud(const Telemetria *t, int64_t ahora) {
    if (!t->valida) return TELEMETRIA_INVALIDA;
    return ahora - t->ms > EDAD_MAX_MS ? TELEMETRIA_ANTIGUA : TELEMETRIA_SANA;
}

/* Llamar con mutex. El registro nunca hace E/S en la ruta de control. */
static Evento *evento(Aplicacion *a, const char *nombre, uint64_t id) {
    if (a->evento_cantidad == EVENTOS) {
        ++a->eventos_perdidos;
        return NULL;
    }
    size_t pos = (a->evento_cabeza + a->evento_cantidad) % EVENTOS;
    a->eventos[pos] = (Evento){
        .ms = ahora_ms() - a->inicio, .id = id,
        .generacion = a->generacion, .pendientes = a->cantidad,
        .direccion = a->direccion, .muestra = a->telemetria.secuencia,
        .edad_telemetria_ms = ahora_ms() - a->telemetria.ms,
        .valida = a->telemetria.valida, .retraso_us = -1, .intervalo_us = -1,
        .jitter_us = 0, .latencia_parada_us = -1, .cpu_hilo_us = -1,
        .politica = -1, .prioridad = -1, .instante_puente_ms = -1
    };
    snprintf(a->eventos[pos].nombre, sizeof a->eventos[pos].nombre, "%s", nombre);
    ++a->evento_cantidad;
    pthread_cond_signal(&a->evento_disponible);
    return &a->eventos[pos];
}

static bool reservar(sem_t *sem) {
    int rc;
    do { rc = sem_trywait(sem); } while (rc != 0 && errno == EINTR);
    if (rc == 0) return true;
    if (errno == EAGAIN) return false;
    perror("sem_trywait");
    abort();
}

static void devolver(sem_t *sem) {
    if (sem_post(sem) != 0) {
        perror("sem_post");
        abort();
    }
}

static void parar(Aplicacion *a, const char *motivo) {
    if (!a->enclavado && (a->direccion != 0 || a->orden_enviada))
        a->solicitud_parada_ns = ahora_ns();
    a->enclavado = true;
    ++a->generacion;
    a->parada_confirmada = false;
    a->puente_armado = false;
    a->rearme_enviado = 0;
    while (a->cantidad != 0) {
        if (!reservar(&a->elementos)) abort();
        --a->cantidad;
        devolver(&a->huecos);
    }
    a->cabeza = 0;
    evento(a, motivo, a->activa_id);
    /* Solo una respuesta del puente puede confirmar la parada. */
}

static void cerrar(Aplicacion *a, const char *motivo) {
    if (!a->cerrando) {
        a->cerrando = true;
        parar(a, motivo);
    }
}

static void registrar_planificacion(Aplicacion *a, const char *nombre) {
    int politica;
    struct sched_param parametros;
    int rc = pthread_getschedparam(pthread_self(), &politica, &parametros);
    pthread_mutex_lock(&a->mutex);
    if (rc != 0) {
        a->error = 1;
        cerrar(a, "error_consulta_planificacion");
    } else {
        Evento *e = evento(a, nombre, 0);
        if (e != NULL) {
            e->politica = politica;
            e->prioridad = parametros.sched_priority;
        }
    }
    pthread_mutex_unlock(&a->mutex);
}

static bool leer_duracion(const char *numero, int maximo, int *valor) {
    char *final;
    errno = 0;
    long n = strtol(numero, &final, 10);
    if (errno != 0 || *final != '\0' || n < 1 || n > maximo) return false;
    *valor = (int)n;
    return true;
}

static void ejecutar_linea(Aplicacion *a, char *linea) {
    char comando[32], extra;
    int duracion = 0;
    /* Primero tokens como cadenas: strtol evita overflow en entradas numericas. */
    char numero[64];
    int campos = sscanf(linea, "%31s %63s %c", comando, numero, &extra);
    if (campos < 1) return;
    pthread_mutex_lock(&a->mutex);
    if (a->cerrando) goto fin;
    if (campos == 1 && strcmp(comando, "salir") == 0) {
        cerrar(a, "salida_solicitada");
    } else if (campos == 1 && strcmp(comando, "parar") == 0) {
        parar(a, "parada_solicitada");
    } else if (campos == 1 && strcmp(comando, "rearmar") == 0) {
        if (a->direccion != 0) evento(a, "rearme_rechazado_en_movimiento", 0);
        else if (!a->parada_confirmada) evento(a, "rearme_rechazado_sin_confirmacion", 0);
        else if (a->sensor != SENSOR_NORMAL || a->fallo_telemetria != TELEMETRIA_SANA ||
                 salud(&a->telemetria, ahora_ms()) != TELEMETRIA_SANA)
            evento(a, "rearme_rechazado_telemetria", 0);
        else {
            a->enclavado = false;
            evento(a, "rearme_solicitado", 0);
        }
    } else if (campos == 1 && strcmp(comando, "estado") == 0) {
        evento(a, a->enclavado ? "estado_enclavado" : "estado_armado", a->activa_id);
    } else if (campos == 2 && strcmp(comando, "sensor") == 0) {
        if (strcmp(numero, "normal") == 0) a->sensor = SENSOR_NORMAL;
        else if (strcmp(numero, "invalido") == 0) a->sensor = SENSOR_INVALIDO;
        else if (strcmp(numero, "congelado") == 0) a->sensor = SENSOR_CONGELADO;
        else { evento(a, "comando_invalido", 0); goto fin; }
        evento(a, "sensor_configurado", 0);
    } else if (campos == 2 && strcmp(comando, "carga") == 0) {
        if (!leer_duracion(numero, CARGA_MAX_MS, &duracion))
            evento(a, "duracion_invalida", 0);
        else {
            a->carga_ms = duracion;
            evento(a, "carga_programada", 0);
        }
    } else if (campos == 2 &&
               (strcmp(comando, "avanzar") == 0 || strcmp(comando, "retroceder") == 0)) {
        if (!leer_duracion(numero, MAX_MS, &duracion)) {
            evento(a, "duracion_invalida", 0);
            goto fin;
        }
        if (a->enclavado) evento(a, "rechazada_enclavamiento", 0);
        else if (!reservar(&a->huecos)) evento(a, "rechazada_cola_llena", 0);
        else {
            Orden orden = {
                .id = ++a->siguiente_id, .generacion = a->generacion,
                .direccion = strcmp(comando, "avanzar") == 0 ? 1 : -1,
                .duracion_ms = duracion, .caduca = ahora_ms() + CADUCIDAD_MS
            };
            a->cola[(a->cabeza + a->cantidad) % CAPACIDAD] = orden;
            ++a->cantidad;
            devolver(&a->elementos);
            evento(a, "orden_aceptada", orden.id);
        }
    } else evento(a, "comando_invalido", 0);
fin:
    pthread_mutex_unlock(&a->mutex);
}

static bool esta_cerrando(Aplicacion *a) {
    pthread_mutex_lock(&a->mutex);
    bool resultado = a->cerrando;
    pthread_mutex_unlock(&a->mutex);
    return resultado;
}

static void *entrada(void *arg) {
    Aplicacion *a = arg;
    registrar_planificacion(a, "planificacion_entrada");
    char linea[128];
    size_t usados = 0;
    bool desbordada = false;
    /* read de un byte evita que stdio oculte lineas al usar poll con tuberias.
       poll con plazo permite cerrar aunque la consola no entregue mas datos. */
    while (!esta_cerrando(a)) {
        struct pollfd fd = {.fd = STDIN_FILENO, .events = POLLIN};
        int rc = poll(&fd, 1, 50);
        if (rc < 0 && errno == EINTR) continue;
        if (rc < 0 || (rc > 0 && (fd.revents & (POLLERR | POLLNVAL)))) {
            pthread_mutex_lock(&a->mutex);
            a->error = 1;
            cerrar(a, "error_entrada");
            pthread_mutex_unlock(&a->mutex);
            break;
        }
        if (rc == 0) continue;
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) {
            if (n == 0 && usados != 0 && !desbordada) {
                linea[usados] = '\0';
                ejecutar_linea(a, linea);
            }
            pthread_mutex_lock(&a->mutex);
            if (n < 0) a->error = 1;
            cerrar(a, n < 0 ? "error_entrada" : "fin_entrada");
            pthread_mutex_unlock(&a->mutex);
            break;
        }
        if (c == '\n') {
            if (!desbordada) {
                linea[usados] = '\0';
                ejecutar_linea(a, linea);
            } else {
                pthread_mutex_lock(&a->mutex);
                evento(a, "linea_demasiado_larga", 0);
                pthread_mutex_unlock(&a->mutex);
            }
            usados = 0;
            desbordada = false;
        } else if (c == '\0') desbordada = true;
        else if (usados < sizeof linea - 1) linea[usados++] = c;
        else desbordada = true;
    }
    return NULL;
}

static void esperar_ciclo(Aplicacion *a, struct timespec *siguiente, long ms) {
    siguiente->tv_nsec += ms * 1000000L;
    siguiente->tv_sec += siguiente->tv_nsec / 1000000000L;
    siguiente->tv_nsec %= 1000000000L;
    int rc;
    do { rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, siguiente, NULL); }
    while (rc == EINTR);
    if (rc != 0) {
        pthread_mutex_lock(&a->mutex);
        a->error = 1;
        cerrar(a, "error_temporizacion");
        pthread_mutex_unlock(&a->mutex);
    }
}

/* Estas funciones se llaman solo desde intercambio, con mutex. Todas las
   operaciones MQ son no bloqueantes. Es el unico productor de ordenes IPC. */
static int enviar_puente(Aplicacion *a, uint32_t tipo, const Orden *orden) {
    int64_t ahora = ahora_ms();
    Mensaje m = {
        .tipo = tipo, .secuencia = a->ipc_secuencia + 1,
        .generacion = a->generacion, .sensor = (uint32_t)a->sensor,
        .instante_ms = ahora, .caduca_ms = ahora + RIDER_LEASE_MS
    };
    unsigned prioridad = 10;
    bool normal = tipo == C_LATIDO || tipo == C_MOVER;
    if (tipo == C_PARAR) prioridad = 30;
    if (tipo == C_REARMAR) prioridad = 20;
    if (tipo == C_SALIR) prioridad = 31;
    if (orden != NULL) {
        prioridad = 0;
        m.orden_id = orden->id;
        m.caduca_ms = orden->caduca;
        m.duracion_ms = (uint32_t)orden->duracion_ms;
        m.direccion = orden->direccion;
    }
    int rc = rider_enviar(a->puente.ordenes, &m, prioridad,
                         normal ? RIDER_NORMAL_MAX : RIDER_MQ_MAX);
    if (rc == 1) ++a->ipc_secuencia;
    if (rc < 0) { a->error = 1; cerrar(a, "error_envio_ipc"); }
    return rc;
}

static Evento *evento_puente(Aplicacion *a, const char *nombre, uint64_t id, const Mensaje *m) {
    Evento *e = evento(a, nombre, id);
    if (e != NULL) e->instante_puente_ms = m->instante_ms;
    return e;
}

static void confirmar_parada(Aplicacion *a, const Mensaje *m) {
    a->direccion = 0;
    a->parada_confirmada = true;
    a->puente_armado = false;
    a->orden_enviada = false;
    Evento *e = evento_puente(a, "parada_confirmada", a->activa_id, m);
    if (a->solicitud_parada_ns != 0) {
        int64_t latencia = (ahora_ns() - a->solicitud_parada_ns) / 1000;
        if (e != NULL) e->latencia_parada_us = latencia;
        if (latencia > a->latencia_parada_max_us) a->latencia_parada_max_us = latencia;
        a->solicitud_parada_ns = 0;
    }
    a->activa_id = 0;
}

static bool recibir_puente(Aplicacion *a, const Mensaje *m, unsigned prioridad) {
    int64_t ahora = ahora_ms();
    if (m->tipo < R_ESTADO || m->tipo > R_SALIDA || m->instante_ms > ahora ||
        m->muestra_ms > ahora || m->muestra == 0 ||
        (m->tipo == R_PARADA && (m->direccion != 0 || (m->flags & RIDER_ARMADO))) ||
        (m->tipo == R_FIN && m->direccion != 0) ||
        (m->tipo == R_SALIDA && (m->direccion != 0 || (m->flags & RIDER_ARMADO))) ||
        (m->tipo == R_REARME && (m->direccion != 0 || !(m->flags & RIDER_ARMADO)))) {
        a->error = 1; cerrar(a, "fallo_protocolo_puente"); return false;
    }
    unsigned clase = prioridad == 0 ? 0 : 1;
    if (m->secuencia <= a->recibidas[clase] || m->caduca_ms <= ahora) return false;
    a->recibidas[clase] = m->secuencia;
    a->ultima_respuesta_ms = ahora;
    if (m->tipo == R_FALLO) {
        a->error = 1; cerrar(a, "fallo_puente"); return false;
    }
    if (m->generacion != a->generacion) return false;
    if (m->muestra > a->telemetria.secuencia) {
        a->telemetria = (Telemetria){
            .secuencia = m->muestra, .ms = m->muestra_ms,
            .valida = (m->flags & RIDER_MUESTRA_VALIDA) != 0
        };
        evento_puente(a, "telemetria", 0, m);
    }
    switch (m->tipo) {
    case R_INICIO:
        if (a->orden_enviada && m->orden_id == a->activa_id && !a->enclavado) {
            if (m->direccion == 0 || m->fin_ms <= m->instante_ms) {
                a->error = 1; cerrar(a, "fallo_protocolo_puente"); break;
            }
            a->direccion = m->direccion;
            a->fin_movimiento = m->fin_ms;
            evento_puente(a, "movimiento_iniciado", m->orden_id, m);
        }
        break;
    case R_FIN:
        if (a->orden_enviada && m->orden_id == a->activa_id) {
            a->direccion = 0;
            a->orden_enviada = false;
            evento_puente(a, "movimiento_completado", m->orden_id, m);
            a->activa_id = 0;
        }
        break;
    case R_PARADA:
        if (a->enclavado && a->parada_enviada == a->generacion) confirmar_parada(a, m);
        break;
    case R_REARME:
        if (!a->enclavado && a->rearme_enviado == a->generacion) {
            a->puente_armado = true;
            evento_puente(a, "rearmado", 0, m);
        }
        break;
    case R_DESCARTE:
        if (a->orden_enviada && m->orden_id == a->activa_id) {
            a->orden_enviada = false;
            evento_puente(a, "orden_descartada", m->orden_id, m);
            a->activa_id = 0;
        } else if (m->orden_id == 0 && !a->enclavado && !a->puente_armado) {
            parar(a, "rearme_rechazado_puente");
        }
        break;
    case R_SALIDA:
        return a->cerrando && a->salida_enviada;
    default: break;
    }
    return false;
}

static void *intercambio(void *arg) {
    Aplicacion *a = arg;
    registrar_planificacion(a, "planificacion_intercambio");
    struct timespec siguiente;
    clock_gettime(CLOCK_MONOTONIC, &siguiente);
    int64_t cierre_ms = 0;
    bool salir = false;
    while (!salir) {
        pthread_mutex_lock(&a->mutex);
        int64_t ahora = ahora_ms();
        for (int i = 0; i < RIDER_MQ_MAX; ++i) {
            Mensaje m;
            unsigned prioridad;
            int rc = rider_recibir(a->puente.respuestas, &m, &prioridad);
            if (rc == 0) break;
            if (rc < 0) {
                a->error = 1;
                cerrar(a, rc == -2 ? "fallo_protocolo_puente" : "error_recepcion_ipc");
                break;
            }
            if (recibir_puente(a, &m, prioridad)) { salir = true; break; }
        }
        int hijo = puente_comprobar(&a->puente);
        if (hijo != 0 && !salir) {
            a->error = 1;
            cerrar(a, "puente_caido");
            salir = true;
        }
        if (!salir && ahora - a->ultima_respuesta_ms > RIDER_TIMEOUT_MS) {
            a->error = 1;
            cerrar(a, "puente_sin_respuesta");
        }
        if (a->cerrando && cierre_ms == 0) cierre_ms = ahora;
        if (!salir && a->enclavado && a->parada_enviada != a->generacion) {
            if (enviar_puente(a, C_PARAR, NULL) == 1) {
                a->parada_enviada = a->generacion;
                a->control_ms = ahora;
            }
        }
        if (!salir && ahora - a->ultimo_latido_ms >= 100) {
            if (enviar_puente(a, C_LATIDO, NULL) == 1) a->ultimo_latido_ms = ahora;
        }
        if (!salir && !a->enclavado && !a->puente_armado && a->rearme_enviado != a->generacion) {
            if (enviar_puente(a, C_REARMAR, NULL) == 1) {
                a->rearme_enviado = a->generacion;
                a->control_ms = ahora;
            }
        }
        bool esperando_control = (a->enclavado && !a->parada_confirmada) ||
                                 (!a->enclavado && !a->puente_armado);
        if (!salir && !a->cerrando && esperando_control && ahora - a->control_ms > RIDER_TIMEOUT_MS) {
            a->error = 1; cerrar(a, "control_puente_sin_confirmacion");
        }
        while (!salir && !a->cerrando && !a->enclavado && a->puente_armado &&
               !a->orden_enviada && a->cantidad != 0 &&
               salud(&a->telemetria, ahora) == TELEMETRIA_SANA) {
            Orden orden = a->cola[a->cabeza];
            bool caducada = orden.generacion != a->generacion || ahora >= orden.caduca;
            if (!caducada && enviar_puente(a, C_MOVER, &orden) != 1) break;
            if (!reservar(&a->elementos)) abort();
            a->cabeza = (a->cabeza + 1) % CAPACIDAD;
            --a->cantidad;
            devolver(&a->huecos);
            if (caducada) { evento(a, "orden_descartada", orden.id); continue; }
            a->orden_enviada = true;
            a->activa_id = orden.id;
            evento(a, "orden_enviada", orden.id);
        }
        if (!salir && a->cerrando && a->parada_confirmada && !a->salida_enviada) {
            if (enviar_puente(a, C_SALIR, NULL) == 1) a->salida_enviada = true;
        }
        if (!salir && cierre_ms != 0 && ahora - cierre_ms >= 500) {
            a->error = 1;
            evento(a, "cierre_puente_sin_confirmacion", 0);
            salir = true;
        }
        pthread_mutex_unlock(&a->mutex);
        if (!salir) esperar_ciclo(a, &siguiente, 10);
    }
    return NULL;
}

static void *supervision(void *arg) {
    Aplicacion *a = arg;
    registrar_planificacion(a, "planificacion_supervision");
    int64_t anterior_ns = 0;
    const int64_t periodo_ns = SUPERVISION_MS * 1000000LL;
    while (!esta_cerrando(a)) {
        siginfo_t info;
        struct timespec plazo = {.tv_sec = 0, .tv_nsec = 100000000};
        int senal = sigtimedwait(&a->senales, &info, &plazo);
        if (senal < 0 && (errno == EINTR || errno == EAGAIN)) continue;
        pthread_mutex_lock(&a->mutex);
        if (senal < 0) {
            a->error = 1;
            cerrar(a, "error_espera_senal");
        } else if (senal == SIGINT || senal == SIGTERM) cerrar(a, "senal_recibida");
        if (a->cerrando) { pthread_mutex_unlock(&a->mutex); break; }
        if (senal != SIGRTMIN || info.si_code != SI_TIMER || info.si_value.sival_ptr != a) {
            pthread_mutex_unlock(&a->mutex);
            continue;
        }
        int carga = a->carga_ms;
        a->carga_ms = 0;
        pthread_mutex_unlock(&a->mutex);
        /* Carga de prueba acotada en este hilo y fuera de la seccion critica. */
        if (carga > 0) {
            int64_t hasta = ahora_ns() + carga * 1000000LL;
            while (ahora_ns() < hasta) { /* Trabajo simulado. */ }
        }
        int64_t instante_ns = ahora_ns();
        int sobrepasos = timer_getoverrun(a->temporizador);
        if (sobrepasos < 0) {
            pthread_mutex_lock(&a->mutex);
            a->error = 1;
            cerrar(a, "error_sobrepasos_timer");
            pthread_mutex_unlock(&a->mutex);
            break;
        }
        /* El sobrepaso corresponde a la senal consumida, no a la pendiente
           que pudiera haberse generado durante la carga de prueba. */
        int64_t expiraciones = (int64_t)sobrepasos + 1;
        pthread_mutex_lock(&a->mutex);
        Telemetria copia = a->telemetria;
        pthread_mutex_unlock(&a->mutex);
        int fallo = salud(&copia, instante_ns / 1000000);
        struct timespec cpu;
        int64_t cpu_us = clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu) == 0
            ? (int64_t)cpu.tv_sec * 1000000 + cpu.tv_nsec / 1000 : -1;
        pthread_mutex_lock(&a->mutex);
        int64_t retraso = (instante_ns - a->siguiente_supervision_ns) / 1000;
        if (retraso < 0) retraso = 0;
        ++a->ciclos;
        a->expiraciones_perdidas += (uint64_t)sobrepasos;
        if (retraso >= SUPERVISION_MS * 1000) ++a->plazos_superados;
        if (retraso > a->retraso_max_us) a->retraso_max_us = retraso;
        Evento *e = evento(a, "ciclo_supervision", 0);
        if (e != NULL) {
            e->retraso_us = retraso;
            e->sobrepasos = sobrepasos;
            e->cpu_hilo_us = cpu_us;
            if (anterior_ns != 0) {
                e->intervalo_us = (instante_ns - anterior_ns) / 1000;
                e->jitter_us = (instante_ns - anterior_ns - expiraciones * periodo_ns) / 1000;
            }
        }
        anterior_ns = instante_ns;
        a->siguiente_supervision_ns += expiraciones * periodo_ns;
        /* Si se publico otra muestra mientras se evaluaba, se usara en el
           proximo ciclo: no se enclava por una copia que ya fue sustituida. */
        if (copia.secuencia == a->telemetria.secuencia && fallo != a->fallo_telemetria) {
            a->fallo_telemetria = fallo;
            if (fallo == TELEMETRIA_SANA) evento(a, "telemetria_recuperada", 0);
            else parar(a, fallo == TELEMETRIA_INVALIDA ? "fallo_telemetria_invalida" : "fallo_telemetria_antigua");
        }
        if (!a->enclavado && a->direccion != 0 && ahora_ms() > a->fin_movimiento + 100)
            parar(a, "fallo_plazo_movimiento");
        pthread_mutex_unlock(&a->mutex);
    }
    return NULL;
}

static void *registro(void *arg) {
    Aplicacion *a = arg;
    registrar_planificacion(a, "planificacion_registro");
    bool fallo = false;
    for (;;) {
        pthread_mutex_lock(&a->mutex);
        while (a->evento_cantidad == 0 && !a->registro_finalizado)
            pthread_cond_wait(&a->evento_disponible, &a->mutex);
        if (a->evento_cantidad == 0 && a->registro_finalizado) {
            pthread_mutex_unlock(&a->mutex);
            break;
        }
        Evento e = a->eventos[a->evento_cabeza];
        a->evento_cabeza = (a->evento_cabeza + 1) % EVENTOS;
        --a->evento_cantidad;
        pthread_mutex_unlock(&a->mutex);
        if (fprintf(a->csv, "%" PRId64 ",%s,%" PRIu64 ",%" PRIu64 ",%zu,%d,"
                    "%" PRIu64 ",%" PRId64 ",%d,%" PRId64 ",%d,%" PRId64
                    ",%" PRId64 ",%" PRId64 ",%d,%d,%" PRId64 ",%" PRId64 "\n",
                    e.ms, e.nombre, e.id, e.generacion, e.pendientes, e.direccion,
                    e.muestra, e.edad_telemetria_ms, e.valida, e.retraso_us,
                    e.sobrepasos, e.intervalo_us, e.jitter_us, e.latencia_parada_us,
                    e.politica, e.prioridad, e.cpu_hilo_us, e.instante_puente_ms) < 0 ||
            fflush(a->csv) != 0) fallo = true;
        if (strcmp(e.nombre, "ciclo_supervision") != 0 && strcmp(e.nombre, "telemetria") != 0)
            printf("[%" PRId64 " ms] %s id=%" PRIu64 " generacion=%" PRIu64
               " pendientes=%zu direccion=%d\n", e.ms, e.nombre, e.id,
               e.generacion, e.pendientes, e.direccion);
        if (fallo) {
            pthread_mutex_lock(&a->mutex);
            a->error = 1;
            cerrar(a, "error_registro");
            pthread_mutex_unlock(&a->mutex);
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    if (argc > 3) {
        fprintf(stderr, "Uso: %s [registro.csv [ejecutable_puente]]\n", argv[0]);
        return 1;
    }
    Aplicacion a = {
        .mutex = PTHREAD_MUTEX_INITIALIZER,
        .evento_disponible = PTHREAD_COND_INITIALIZER,
        .enclavado = true, .generacion = 1,
        .puente = {.ordenes = (mqd_t)-1, .respuestas = (mqd_t)-1, .pid = -1}
    };
    a.inicio = ahora_ms();
    bool temporizador_creado = false;
    bool puente_creado = false;
    a.csv = fopen(argc >= 2 ? argv[1] : "rider_sim.csv", "w");
    if (a.csv == NULL) { perror("registro CSV"); return 1; }
    if (fprintf(a.csv, "ms,evento,id,generacion,pendientes,direccion,muestra,"
                "edad_telemetria_ms,valida,retraso_us,expiraciones_perdidas,"
                "intervalo_us,jitter_us,latencia_parada_us,politica,prioridad,cpu_hilo_us,instante_puente_ms\n") < 0 ||
        fflush(a.csv) != 0) { perror("cabecera CSV"); fclose(a.csv); return 1; }
    if (sem_init(&a.huecos, 0, CAPACIDAD) != 0) {
        perror("sem_init huecos"); fclose(a.csv); return 1;
    }
    if (sem_init(&a.elementos, 0, 0) != 0) {
        perror("sem_init elementos"); sem_destroy(&a.huecos); fclose(a.csv); return 1;
    }
    /* Todos los hilos heredan la mascara. Solo supervision consume estas
       senales con sigtimedwait: no se ejecutan manejadores asincronos. */
    sigemptyset(&a.senales);
    sigaddset(&a.senales, SIGINT);
    sigaddset(&a.senales, SIGTERM);
    sigaddset(&a.senales, SIGRTMIN);
    int rc_mascara = pthread_sigmask(SIG_BLOCK, &a.senales, NULL);
    if (rc_mascara != 0) {
        fprintf(stderr, "pthread_sigmask: %s\n", strerror(rc_mascara));
        a.error = 1;
        goto limpiar;
    }
    char ruta_puente[4096];
    const char *ejecutable_puente = argc == 3 ? argv[2] : ruta_puente;
    if (argc != 3) {
        /* Resolver junto al ejecutable, tambien al lanzarlo desde otra carpeta. */
        ssize_t n = readlink("/proc/self/exe", ruta_puente, sizeof ruta_puente - 1);
        if (n < 0 || (size_t)n >= sizeof ruta_puente - 1) {
            perror("ruta del ejecutable"); a.error = 1; goto limpiar;
        }
        ruta_puente[n] = '\0';
        char *barra = strrchr(ruta_puente, '/');
        size_t disponibles = sizeof ruta_puente - (size_t)(barra + 1 - ruta_puente);
        if (snprintf(barra + 1, disponibles, "rider_bridge_sim") >= (int)disponibles) {
            fprintf(stderr, "Ruta del puente demasiado larga\n"); a.error = 1; goto limpiar;
        }
    }
    Mensaje listo;
    if (puente_iniciar(&a.puente, ejecutable_puente, &listo) != 0) {
        perror("inicio del puente"); a.error = 1; goto limpiar;
    }
    puente_creado = true;
    a.telemetria = (Telemetria){
        .secuencia = listo.muestra, .ms = listo.muestra_ms,
        .valida = (listo.flags & RIDER_MUESTRA_VALIDA) != 0
    };
    a.parada_confirmada = true;
    a.parada_enviada = a.generacion;
    a.recibidas[1] = listo.secuencia;
    a.ultima_respuesta_ms = a.control_ms = ahora_ms();
    evento(&a, "puente_iniciado", (uint64_t)a.puente.pid);
    struct sigevent notificacion = {
        .sigev_notify = SIGEV_SIGNAL, .sigev_signo = SIGRTMIN,
        .sigev_value = {.sival_ptr = &a}
    };
    if (timer_create(CLOCK_MONOTONIC, &notificacion, &a.temporizador) != 0) {
        perror("timer_create"); a.error = 1; goto limpiar;
    }
    temporizador_creado = true;
    a.siguiente_supervision_ns = ahora_ns() + SUPERVISION_MS * 1000000LL;
    struct itimerspec periodo = {
        .it_interval = {.tv_sec = 0, .tv_nsec = SUPERVISION_MS * 1000000L},
        .it_value = {
            .tv_sec = (time_t)(a.siguiente_supervision_ns / 1000000000),
            .tv_nsec = (long)(a.siguiente_supervision_ns % 1000000000)
        }
    };
    if (timer_settime(a.temporizador, TIMER_ABSTIME, &periodo, NULL) != 0) {
        perror("timer_settime"); a.error = 1; goto limpiar;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    puts("Rider SIMULADO. Inicialmente parado y enclavado. Sin conexion al robot.");
    printf("Puente simulado: pid=%ld ordenes=%s respuestas=%s\n", (long)a.puente.pid,
           a.puente.nombre_ordenes, a.puente.nombre_respuestas);
    puts("Ordenes: rearmar | avanzar MS | retroceder MS | parar | estado | salir");
    puts("Pruebas simuladas: sensor normal|invalido|congelado | carga MS (1..200)");
    puts("Duracion: 1..3000 ms; caducidad en cola: 2000 ms; Ctrl+C o EOF cierran.");
    pthread_t hilos[4];
    void *(*tareas[4])(void *) = {registro, intercambio, supervision, entrada};
    size_t creados = 0;
    for (; creados < 4; ++creados) {
        int rc = pthread_create(&hilos[creados], NULL, tareas[creados], &a);
        if (rc != 0) {
            fprintf(stderr, "pthread_create: %s\n", strerror(rc));
            pthread_mutex_lock(&a.mutex);
            a.error = 1;
            cerrar(&a, "error_creacion_hilo");
            pthread_mutex_unlock(&a.mutex);
            break;
        }
    }
    /* El registro se une al final para vaciar los eventos de los otros hilos. */
    for (size_t i = 1; i < creados; ++i) pthread_join(hilos[i], NULL);
    int resultado_puente = puente_finalizar(&a.puente);
    puente_creado = false;
    pthread_mutex_lock(&a.mutex);
    if (resultado_puente != 0) a.error = 1;
    evento(&a, resultado_puente == 0 ? "puente_recogido" : "puente_recogido_con_error", 0);
    evento(&a, "cierre_completado", 0);
    a.registro_finalizado = true;
    pthread_cond_signal(&a.evento_disponible);
    pthread_mutex_unlock(&a.mutex);
    if (creados > 0) pthread_join(hilos[0], NULL);
    printf("Medidas: ciclos=%" PRIu64 " expiraciones_perdidas=%" PRIu64
           " plazos_superados=%" PRIu64 " retraso_max_us=%" PRId64
           " latencia_parada_max_us=%" PRId64 ".\n",
           a.ciclos, a.expiraciones_perdidas, a.plazos_superados,
           a.retraso_max_us, a.latencia_parada_max_us);
    printf("Fin: %zu hilos unidos; eventos perdidos=%lu.\n", creados, a.eventos_perdidos);
limpiar:
    if (puente_creado && puente_finalizar(&a.puente) != 0) a.error = 1;
    if (temporizador_creado && timer_delete(a.temporizador) != 0) {
        perror("timer_delete"); a.error = 1;
    }
    sem_destroy(&a.elementos);
    sem_destroy(&a.huecos);
    pthread_cond_destroy(&a.evento_disponible);
    pthread_mutex_destroy(&a.mutex);
    if (fclose(a.csv) != 0) { perror("cierre CSV"); a.error = 1; }
    return a.error ? 1 : 0;
}
