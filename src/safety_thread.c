#include "safety.h"

#include "metrics.h"
#include "robot_config.h"
#include "scheduler_utils.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

static int safety_loop(timer_t timer, const sigset_t *signals)
{
    const struct itimerspec period = {
        .it_value = { .tv_nsec = SAFETY_PERIOD_NS },
        .it_interval = { .tv_nsec = SAFETY_PERIOD_NS }
    };
    if (timer_settime(timer, 0, &period, NULL) == -1)
        return -1;

    const struct timespec timeout = { .tv_nsec = 2 * SAFETY_PERIOD_NS };
    uint64_t previous = 0;
    uint64_t cycle = 0;
    while (shared_state_snapshot().safety != SAFETY_SHUTDOWN) {
        int signal_number = sigtimedwait(signals, NULL, &timeout);
        if (signal_number == -1) {
            if (errno == EAGAIN || errno == EINTR)
                continue;
            return -1;
        }
        uint64_t now = monotonic_ns();
        int overruns = timer_getoverrun(timer);
        if (now == 0 || overruns == -1)
            return -1;

        shared_state_evaluate();
        SafetyMetric sample = {
            .state = shared_state_snapshot(),
            .timestamp_ns = now,
            .cycle = cycle++,
            .period_ns = previous ? now - previous : 0,
            .overruns = overruns
        };
        metrics_publish(&sample);
        previous = now;
    }
    SafetyMetric shutdown = {
        .state = shared_state_snapshot(),
        .timestamp_ns = monotonic_ns(),
        .cycle = cycle
    };
    metrics_publish(&shutdown);
    return 0;
}

void *safety_worker(void *argument)
{
    configure_safety_scheduler(*(const bool *)argument);
    int timer_signal = SIGRTMIN;
    sigset_t signals;
    sigemptyset(&signals);
    sigaddset(&signals, timer_signal);
    struct sigevent event = { .sigev_notify = SIGEV_SIGNAL, .sigev_signo = timer_signal };
    timer_t timer;
    if (timer_create(CLOCK_MONOTONIC, &event, &timer) == -1) {
        perror("Creating safety timer");
        shared_state_fail();
        return NULL;
    }
    if (safety_loop(timer, &signals) == -1) {
        perror("Safety timer");
        shared_state_fail();
    }
    timer_delete(timer);
    return NULL;
}
