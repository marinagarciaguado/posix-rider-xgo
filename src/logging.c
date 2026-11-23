#include "metrics.h"

#include "robot_config.h"
#include "safety.h"

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>

static struct {
    pthread_mutex_t lock;
    pthread_cond_t ready;
    pthread_t thread;
    FILE *file;
    SafetyMetric samples[METRICS_CAPACITY];
    size_t head;
    size_t count;
    uint64_t dropped;
    bool stopping;
    bool write_failed;
} log_queue = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .ready = PTHREAD_COND_INITIALIZER
};

/* Only the safety worker publishes samples. */
void metrics_publish(const SafetyMetric *sample)
{
    if (pthread_mutex_trylock(&log_queue.lock) != 0) {
        ++log_queue.dropped;
        return;
    }
    if (log_queue.count == METRICS_CAPACITY) {
        ++log_queue.dropped;
    } else {
        size_t tail = (log_queue.head + log_queue.count) % METRICS_CAPACITY;
        log_queue.samples[tail] = *sample;
        ++log_queue.count;
        pthread_cond_signal(&log_queue.ready);
    }
    pthread_mutex_unlock(&log_queue.lock);
}

static int64_t age_ms(uint64_t now, uint64_t then)
{
    return then != 0 && now >= then ? (int64_t)((now - then) / 1000000) : -1;
}

static int write_sample(const SafetyMetric *sample)
{
    const RobotStateSnapshot *state = &sample->state;
    int64_t jitter = sample->period_ns ? (int64_t)sample->period_ns - SAFETY_PERIOD_NS : 0;
    int64_t dispatch_latency = state->estop_sent_ns && state->estop_sent_ns >= state->fault_ns ?
                              (int64_t)(state->estop_sent_ns - state->fault_ns) : -1;
    return fprintf(log_queue.file,
                   "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRId64 ",%d,%d,"
                   "%.3f,%.3f,%.3f,%d,%" PRId64 ",%" PRId64 ",%d,%s,%s,%" PRId64 "\n",
                   sample->timestamp_ns, sample->cycle, sample->period_ns, jitter,
                   sample->overruns, jitter > DEADLINE_TOLERANCE_NS || sample->overruns > 0,
                   (double)state->telemetry.pitch_deg, (double)state->telemetry.roll_deg,
                   (double)state->telemetry.battery_pct,
                   (state->telemetry.status_flags & XGO_STATUS_SENSOR_VALID) != 0,
                   age_ms(sample->timestamp_ns, state->telemetry.timestamp_ns),
                   age_ms(sample->timestamp_ns, state->operator_rx_ns), state->child_alive,
                   safety_state_name(state->safety), fault_name(state->fault), dispatch_latency);
}

static void *log_worker(void *unused)
{
    (void)unused;
    pthread_mutex_lock(&log_queue.lock);
    for (;;) {
        while (log_queue.count == 0 && !log_queue.stopping)
            pthread_cond_wait(&log_queue.ready, &log_queue.lock);
        if (log_queue.count == 0)
            break;
        SafetyMetric sample = log_queue.samples[log_queue.head];
        log_queue.head = (log_queue.head + 1) % METRICS_CAPACITY;
        --log_queue.count;
        pthread_mutex_unlock(&log_queue.lock);

        if (write_sample(&sample) < 0 || fflush(log_queue.file) == EOF) {
            log_queue.write_failed = true;
            perror("Writing metrics");
            return NULL;
        }
        pthread_mutex_lock(&log_queue.lock);
    }
    pthread_mutex_unlock(&log_queue.lock);
    return NULL;
}

int metrics_start(const char *path)
{
    log_queue.file = fopen(path, "w");
    if (log_queue.file == NULL)
        return errno;
    if (fputs("timestamp_ns,cycle_index,observed_period_ns,jitter_ns,timer_overrun_count,"
              "deadline_miss,pitch_deg,roll_deg,battery_pct,sensor_valid,telemetry_age_ms,operator_age_ms,"
              "child_alive,safety_state,fault_code,emergency_dispatch_latency_ns\n",
              log_queue.file) == EOF || fflush(log_queue.file) == EOF) {
        int error = errno;
        fclose(log_queue.file);
        return error;
    }
    int error = pthread_create(&log_queue.thread, NULL, log_worker, NULL);
    if (error != 0)
        fclose(log_queue.file);
    return error;
}

int metrics_stop(void)
{
    pthread_mutex_lock(&log_queue.lock);
    log_queue.stopping = true;
    pthread_cond_signal(&log_queue.ready);
    pthread_mutex_unlock(&log_queue.lock);
    pthread_join(log_queue.thread, NULL);
    if (log_queue.dropped != 0)
        fprintf(stderr, "Dropped metrics samples: %" PRIu64 "\n", log_queue.dropped);
    int error = fclose(log_queue.file);
    return error == EOF || log_queue.write_failed ? -1 : 0;
}
