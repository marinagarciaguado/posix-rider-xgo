#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

typedef struct {
    pthread_mutex_t mutex;
    int dato;
} Estado;

static void sumar_ms(struct timespec *t, long ms) {
    t->tv_nsec += (ms % 1000) * 1000000L;
    t->tv_sec  += ms / 1000 + t->tv_nsec / 1000000000L;
    t->tv_nsec %= 1000000000L;
}

static void esperar_hasta(const struct timespec *t) {
    int err;
    do {
        err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
    } while (err == EINTR);
    if (err != 0) fprintf(stderr, "Error en clock_nanosleep: %d\n", err);
}

static void *tarea_sensores(void *arg) {
    Estado *e = arg;
    struct timespec siguiente;
    clock_gettime(CLOCK_MONOTONIC, &siguiente);
    for (int n = 1; n <= 10; ++n) {
        pthread_mutex_lock(&e->mutex);
        e->dato = n; // Valor SIMULADO: no accede al robot.
        pthread_mutex_unlock(&e->mutex);
        sumar_ms(&siguiente, 200); // Periodo: 200 ms.
        esperar_hasta(&siguiente);
    }
    return NULL;
}

static void *tarea_supervision(void *arg) {
    Estado *e = arg;
    struct timespec siguiente;
    clock_gettime(CLOCK_MONOTONIC, &siguiente);
    for (int k = 1; k <= 5; ++k) {
        int copia;
        pthread_mutex_lock(&e->mutex);
        copia = e->dato;
        pthread_mutex_unlock(&e->mutex);
        printf("[Supervision] ciclo=%d, muestra=%d\n", k, copia);
        sumar_ms(&siguiente, 500); // Periodo: 500 ms.
        esperar_hasta(&siguiente);
    }
    return NULL;
}

int main(void) {
    Estado estado = {.mutex = PTHREAD_MUTEX_INITIALIZER, .dato = 0};
    pthread_t t_sensores, t_supervision;
    if (pthread_create(&t_sensores, NULL, tarea_sensores, &estado) != 0) {
        fprintf(stderr, "No se pudo iniciar sensores\n");
        return 1;
    }
    if (pthread_create(&t_supervision, NULL, tarea_supervision, &estado) != 0) {
        fprintf(stderr, "No se pudo iniciar supervision\n");
        pthread_join(t_sensores, NULL);
        pthread_mutex_destroy(&estado.mutex);
        return 1;
    }
    pthread_join(t_sensores, NULL);
    pthread_join(t_supervision, NULL);
    pthread_mutex_destroy(&estado.mutex);
    puts("Fin: ambos hilos han terminado.");
    return 0;
}
