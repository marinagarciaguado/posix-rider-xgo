#define _POSIX_C_SOURCE 200809L
#include "rider_ipc.h"
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

int main(void) {
    char nombre[80];
    snprintf(nombre, sizeof nombre, "/rider_test_%ld", (long)getpid());
    struct mq_attr attr = {.mq_maxmsg = RIDER_MQ_MAX, .mq_msgsize = RIDER_WIRE_SIZE};
    mqd_t q = mq_open(nombre, O_CREAT | O_EXCL | O_RDWR | O_NONBLOCK, 0600, &attr);
    assert(q != (mqd_t)-1);
    /* Desenlazar ahora evita residuos incluso si falla una asercion. */
    assert(mq_unlink(nombre) == 0);
    Mensaje m = {.tipo = C_MOVER, .secuencia = 1, .generacion = 1};
    for (int i = 0; i < RIDER_NORMAL_MAX; ++i)
        assert(rider_enviar(q, &m, 0, RIDER_NORMAL_MAX) == 1);
    assert(rider_enviar(q, &m, 0, RIDER_NORMAL_MAX) == 0);
    m.tipo = C_PARAR;
    m.generacion = 2;
    assert(rider_enviar(q, &m, 30, RIDER_MQ_MAX) == 1);
    Mensaje recibida;
    unsigned prioridad;
    assert(rider_recibir(q, &recibida, &prioridad) == 1);
    assert(recibida.tipo == C_PARAR && prioridad == 30 && recibida.generacion == 2);
    for (int i = 0; i < RIDER_NORMAL_MAX; ++i) {
        assert(rider_recibir(q, &recibida, &prioridad) == 1);
        assert(recibida.tipo == C_MOVER && prioridad == 0);
    }
    assert(rider_recibir(q, &recibida, &prioridad) == 0);
    mq_close(q);
    puts("IPC: reserva de capacidad, prioridad de parada y recepcion no bloqueante OK");
    return 0;
}
