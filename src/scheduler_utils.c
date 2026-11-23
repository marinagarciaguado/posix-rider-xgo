#include "scheduler_utils.h"

#include "ipc_protocol.h"

#include <sched.h>
#include <stdio.h>
#include <string.h>

uint64_t monotonic_ns(void)
{
    uint64_t now = 0;
    xgo_monotonic_now_ns(&now);
    return now;
}

bool timestamp_fresh(uint64_t now, uint64_t stamp, unsigned max_ms)
{
    return stamp != 0 && now >= stamp && now - stamp <= max_ms * NS_PER_MS;
}

bool sequence_newer(uint32_t value, uint32_t previous)
{
    uint32_t difference = value - previous;
    return difference != 0 && difference < UINT32_C(0x80000000);
}

int init_shared_mutex(pthread_mutex_t *mutex)
{
    pthread_mutexattr_t attr;
    int error = pthread_mutexattr_init(&attr);
    if (error != 0)
        return error;

    error = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    if (error == 0)
        error = pthread_mutex_init(mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    if (error != 0) {
        fprintf(stderr, "Priority inheritance unavailable: %s\n", strerror(error));
        error = pthread_mutex_init(mutex, NULL);
    }
    return error;
}

void configure_safety_scheduler(bool use_fifo)
{
    int policy;
    struct sched_param param = { .sched_priority = sched_get_priority_min(SCHED_FIFO) };
    if (use_fifo) {
        int error = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (error != 0)
            fprintf(stderr, "SCHED_FIFO request failed: %s\n", strerror(error));
    }
    if (pthread_getschedparam(pthread_self(), &policy, &param) == 0)
        fprintf(stderr, "Safety scheduler: %s, priority %d\n",
                policy == SCHED_FIFO ? "SCHED_FIFO" :
                policy == SCHED_OTHER ? "SCHED_OTHER" : "other", param.sched_priority);
}
