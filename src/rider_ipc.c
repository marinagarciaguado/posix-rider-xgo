#define _POSIX_C_SOURCE 200809L
#include "rider_ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static int64_t ms(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) return -1;
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}
static void pausa(void) {
    struct timespec t = {.tv_nsec = 10000000};
    while (nanosleep(&t, &t) != 0 && errno == EINTR) { }
}
int rider_enviar(mqd_t cola, const Mensaje *m, unsigned prioridad, long limite) {
    struct mq_attr attr;
    if (mq_getattr(cola, &attr) != 0) return -1;
    if (attr.mq_curmsgs >= limite) return 0;
    unsigned char datos[RIDER_WIRE_SIZE];
    rider_codificar(m, datos);
    int rc;
    do { rc = mq_send(cola, (const char *)datos, sizeof datos, prioridad); }
    while (rc != 0 && errno == EINTR);
    if (rc == 0) return 1;
    return errno == EAGAIN ? 0 : -1;
}
int rider_recibir(mqd_t cola, Mensaje *m, unsigned *prioridad) {
    unsigned char datos[RIDER_WIRE_SIZE];
    ssize_t n;
    do { n = mq_receive(cola, (char *)datos, sizeof datos, prioridad); }
    while (n < 0 && errno == EINTR);
    if (n < 0) return errno == EAGAIN ? 0 : -1;
    return rider_decodificar(datos, (size_t)n, m) ? 1 : -2;
}
int puente_comprobar(Puente *p) {
    if (p->recogido || p->pid <= 0) return 1;
    pid_t rc;
    do { rc = waitpid(p->pid, &p->estado, WNOHANG); } while (rc < 0 && errno == EINTR);
    if (rc == p->pid) { p->recogido = true; return 1; }
    return rc < 0 ? -1 : 0;
}
static bool esperar_hijo(Puente *p, int plazo_ms) {
    int64_t fin = ms() + plazo_ms;
    while (ms() < fin) {
        int rc = puente_comprobar(p);
        if (rc != 0) return rc == 1;
        pausa();
    }
    return puente_comprobar(p) == 1;
}
int puente_finalizar(Puente *p) {
    int resultado = 0;
    if (p->pid > 0) {
        if (!esperar_hijo(p, 250)) {
            resultado = -1;
            /* SIGCONT permite terminar tambien un simulador detenido con SIGSTOP. */
            kill(p->pid, SIGCONT);
            kill(p->pid, SIGTERM);
            if (!esperar_hijo(p, 250)) {
                kill(p->pid, SIGKILL);
                pid_t rc;
                do { rc = waitpid(p->pid, &p->estado, 0); } while (rc < 0 && errno == EINTR);
                p->recogido = rc == p->pid;
            }
        }
        if (!p->recogido || !WIFEXITED(p->estado) || WEXITSTATUS(p->estado) != 0)
            resultado = -1;
    }
    if (p->ordenes != (mqd_t)-1 && mq_close(p->ordenes) != 0) resultado = -1;
    if (p->respuestas != (mqd_t)-1 && mq_close(p->respuestas) != 0) resultado = -1;
    p->ordenes = p->respuestas = (mqd_t)-1;
    if (p->nombre_ordenes[0] && mq_unlink(p->nombre_ordenes) != 0) resultado = -1;
    if (p->nombre_respuestas[0] && mq_unlink(p->nombre_respuestas) != 0) resultado = -1;
    p->nombre_ordenes[0] = p->nombre_respuestas[0] = '\0';
    return resultado;
}
int puente_iniciar(Puente *p, const char *ruta, Mensaje *listo) {
    *p = (Puente){.ordenes = (mqd_t)-1, .respuestas = (mqd_t)-1, .pid = -1};
    char ordenes[96], respuestas[96];
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) return -1;
    snprintf(ordenes, sizeof ordenes, "/rider_%ld_%ld_o", (long)getpid(), t.tv_nsec);
    snprintf(respuestas, sizeof respuestas, "/rider_%ld_%ld_r", (long)getpid(), t.tv_nsec);
    struct mq_attr attr = {.mq_maxmsg = RIDER_MQ_MAX, .mq_msgsize = RIDER_WIRE_SIZE};
    p->ordenes = mq_open(ordenes, O_CREAT | O_EXCL | O_RDWR | O_NONBLOCK, 0600, &attr);
    if (p->ordenes == (mqd_t)-1) return -1;
    strcpy(p->nombre_ordenes, ordenes); /* Solo se desenlaza lo que hemos creado. */
    p->respuestas = mq_open(respuestas, O_CREAT | O_EXCL | O_RDWR | O_NONBLOCK, 0600, &attr);
    if (p->respuestas == (mqd_t)-1) goto fallo;
    strcpy(p->nombre_respuestas, respuestas);
    p->pid = fork(); /* Antes de crear cualquier hilo del supervisor. */
    if (p->pid < 0) goto fallo;
    if (p->pid == 0) {
        execl(ruta, ruta, ordenes, respuestas, (char *)NULL);
        _exit(127);
    }
    int64_t fin = ms() + 1500;
    while (ms() < fin) {
        unsigned prioridad;
        int rc = rider_recibir(p->respuestas, listo, &prioridad);
        if (rc == 1 && listo->tipo == R_LISTO && listo->generacion == 1 &&
            listo->direccion == 0 && listo->flags == RIDER_MUESTRA_VALIDA && listo->muestra != 0 &&
            listo->muestra_ms <= ms() && listo->caduca_ms > ms()) return 0;
        if (rc != 0 || puente_comprobar(p) != 0) { errno = EPROTO; goto fallo; }
        pausa();
    }
    errno = ETIMEDOUT;
fallo: {
        int error = errno;
        puente_finalizar(p);
        errno = error;
        return -1;
    }
}
