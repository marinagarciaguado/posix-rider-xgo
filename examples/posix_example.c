#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

typedef struct {
    pthread_mutex_t mutex;
    int value;
} State;

static void add_ms(struct timespec *t, long ms) {
    t->tv_nsec += (ms % 1000) * 1000000L;
    t->tv_sec  += ms / 1000 + t->tv_nsec / 1000000000L;
    t->tv_nsec %= 1000000000L;
}

static void wait_until(const struct timespec *t) {
    int err;
    do {
        err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
    } while (err == EINTR);
    if (err != 0) fprintf(stderr, "clock_nanosleep failed: %d\n", err);
}

static void *sensor_task(void *arg) {
    State *e = arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int n = 1; n <= 10; ++n) {
        pthread_mutex_lock(&e->mutex);
        e->value = n; // SIMULATED value: no robot access.
        pthread_mutex_unlock(&e->mutex);
        add_ms(&next, 200); // Period: 200 ms.
        wait_until(&next);
    }
    return NULL;
}

static void *supervisor_task(void *arg) {
    State *e = arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (int k = 1; k <= 5; ++k) {
        int snapshot;
        pthread_mutex_lock(&e->mutex);
        snapshot = e->value;
        pthread_mutex_unlock(&e->mutex);
        printf("[Supervisor] cycle=%d, sample=%d\n", k, snapshot);
        add_ms(&next, 500); // Period: 500 ms.
        wait_until(&next);
    }
    return NULL;
}

int main(void) {
    State state = {.mutex = PTHREAD_MUTEX_INITIALIZER, .value = 0};
    pthread_t sensor_thread, supervisor_thread;
    if (pthread_create(&sensor_thread, NULL, sensor_task, &state) != 0) {
        fprintf(stderr, "Could not start sensor thread\n");
        return 1;
    }
    if (pthread_create(&supervisor_thread, NULL, supervisor_task, &state) != 0) {
        fprintf(stderr, "Could not start supervisor thread\n");
        pthread_join(sensor_thread, NULL);
        pthread_mutex_destroy(&state.mutex);
        return 1;
    }
    pthread_join(sensor_thread, NULL);
    pthread_join(supervisor_thread, NULL);
    pthread_mutex_destroy(&state.mutex);
    puts("Done: both threads have finished.");
    return 0;
}
