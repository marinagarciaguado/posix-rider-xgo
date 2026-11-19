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

/* Simulation only: no function communicates with hardware. */
/* These simulation parameters are implementation choices, not verified robot
   limits or requirements from the original assignment. */
enum {
    QUEUE_CAPACITY = 8, EVENT_CAPACITY = 256, MAX_MS = RIDER_MAX_MS, COMMAND_TTL_MS = 2000,
    SUPERVISION_MS = 20, MAX_SAMPLE_AGE_MS = RIDER_MAX_SAMPLE_AGE_MS, MAX_LOAD_MS = 200
};
enum { SENSOR_NORMAL, SENSOR_INVALID, SENSOR_FROZEN };
enum { TELEMETRY_HEALTHY, TELEMETRY_INVALID, TELEMETRY_STALE };
typedef struct {
    uint64_t sequence;
    int64_t ms;
    bool valid;
} Telemetry;
typedef struct {
    uint64_t id, generation;
    int direction, duration_ms;
    int64_t expires_ms;
} Command;
typedef struct {
    int64_t ms;
    uint64_t id, generation;
    size_t pending;
    int direction;
    uint64_t sample;
    int64_t sample_age_ms, lateness_us, interval_us, jitter_us;
    int64_t stop_latency_us, thread_cpu_us, bridge_timestamp_ms;
    int valid, overruns, policy, priority;
    char name[48];
} Event;
typedef struct {
    /* All semaphore reservations use trywait while holding this mutex.
       Queue draining cannot race with reservations outside the lock. */
    pthread_mutex_t mutex;
    pthread_cond_t event_available;
    sem_t slots, items;
    Command queue[QUEUE_CAPACITY];
    size_t head, count;
    uint64_t next_id, generation;
    bool latched, shutting_down, logger_done;
    int direction, error;
    uint64_t active_id;
    int64_t movement_end_ms, start_ms;
    Telemetry telemetry;
    int sensor, telemetry_fault, load_ms;
    int64_t stop_requested_ns;
    Bridge bridge;
    bool bridge_armed, stop_confirmed, command_sent, quit_sent;
    uint64_t ipc_sequence, received_sequences[2], stop_sent_generation, arm_sent_generation;
    int64_t last_response_ms, last_heartbeat_ms, control_ms;
    timer_t timer;
    sigset_t signals;
    int64_t next_supervision_ns;
    uint64_t cycles, missed_expirations, late_cycles;
    int64_t max_lateness_us, max_stop_latency_us;
    Event events[EVENT_CAPACITY];
    size_t event_head, event_count;
    unsigned long dropped_events;
    FILE *csv;
} Application;

static int64_t monotonic_ns(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
        perror("clock_gettime");
        abort();
    }
    return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

static int64_t monotonic_ms(void) {
    return monotonic_ns() / 1000000;
}

static int telemetry_health(const Telemetry *t, int64_t now) {
    if (!t->valid) return TELEMETRY_INVALID;
    return now - t->ms > MAX_SAMPLE_AGE_MS ? TELEMETRY_STALE : TELEMETRY_HEALTHY;
}

/* Requires the application mutex. File I/O belongs to the logger thread. */
static Event *record_event(Application *app, const char *name, uint64_t id) {
    if (app->event_count == EVENT_CAPACITY) {
        ++app->dropped_events;
        return NULL;
    }
    size_t pos = (app->event_head + app->event_count) % EVENT_CAPACITY;
    app->events[pos] = (Event){
        .ms = monotonic_ms() - app->start_ms, .id = id,
        .generation = app->generation, .pending = app->count,
        .direction = app->direction, .sample = app->telemetry.sequence,
        .sample_age_ms = monotonic_ms() - app->telemetry.ms,
        .valid = app->telemetry.valid, .lateness_us = -1, .interval_us = -1,
        .jitter_us = 0, .stop_latency_us = -1, .thread_cpu_us = -1,
        .policy = -1, .priority = -1, .bridge_timestamp_ms = -1
    };
    snprintf(app->events[pos].name, sizeof app->events[pos].name, "%s", name);
    ++app->event_count;
    pthread_cond_signal(&app->event_available);
    return &app->events[pos];
}

static bool try_acquire(sem_t *sem) {
    int rc;
    do { rc = sem_trywait(sem); } while (rc != 0 && errno == EINTR);
    if (rc == 0) return true;
    if (errno == EAGAIN) return false;
    perror("sem_trywait");
    abort();
}

static void release_slot(sem_t *sem) {
    if (sem_post(sem) != 0) {
        perror("sem_post");
        abort();
    }
}

static void request_stop(Application *app, const char *reason) {
    bool movement_pending = app->direction != 0 || app->command_sent;
    if (!app->latched && movement_pending)
        app->stop_requested_ns = monotonic_ns();
    app->latched = true;
    ++app->generation;
    app->stop_confirmed = false;
    app->bridge_armed = false;
    app->arm_sent_generation = 0;
    while (app->count != 0) {
        if (!try_acquire(&app->items)) abort();
        --app->count;
        release_slot(&app->slots);
    }
    app->head = 0;
    record_event(app, reason, app->active_id);
    /* Only a bridge response can confirm a stop. */
}

static void request_shutdown(Application *app, const char *reason) {
    if (!app->shutting_down) {
        app->shutting_down = true;
        request_stop(app, reason);
    }
}

static void record_scheduling(Application *app, const char *name) {
    int policy;
    struct sched_param parameters;
    int rc = pthread_getschedparam(pthread_self(), &policy, &parameters);
    pthread_mutex_lock(&app->mutex);
    if (rc != 0) {
        app->error = 1;
        request_shutdown(app, "scheduling_query_failed");
    } else {
        Event *e = record_event(app, name, 0);
        if (e != NULL) {
            e->policy = policy;
            e->priority = parameters.sched_priority;
        }
    }
    pthread_mutex_unlock(&app->mutex);
}

static bool parse_duration(const char *number, int maximum, int *value) {
    char *end;
    errno = 0;
    long n = strtol(number, &end, 10);
    if (errno != 0) return false;
    if (*end != '\0') return false;
    if (n < 1 || n > maximum) return false;
    *value = (int)n;
    return true;
}

static bool telemetry_allows_arm(const Application *app) {
    if (app->sensor != SENSOR_NORMAL) return false;
    if (app->telemetry_fault != TELEMETRY_HEALTHY) return false;
    return telemetry_health(&app->telemetry, monotonic_ms()) == TELEMETRY_HEALTHY;
}

static void handle_arm(Application *app, const char *argument) {
    (void)argument;
    if (app->direction != 0) {
        record_event(app, "arm_rejected_moving", 0);
        return;
    }
    if (!app->stop_confirmed) {
        record_event(app, "arm_rejected_unconfirmed_stop", 0);
        return;
    }
    if (!telemetry_allows_arm(app)) {
        record_event(app, "arm_rejected_telemetry", 0);
        return;
    }
    app->latched = false;
    record_event(app, "arm_requested", 0);
}

static void handle_sensor(Application *app, const char *argument) {
    const char *modes[] = {"normal", "invalid", "frozen"};
    for (int mode = SENSOR_NORMAL; mode <= SENSOR_FROZEN; ++mode) {
        if (strcmp(argument, modes[mode]) != 0) continue;
        app->sensor = mode;
        record_event(app, "sensor_configured", 0);
        return;
    }
    record_event(app, "invalid_command", 0);
}

static void handle_load(Application *app, const char *argument) {
    int duration;
    if (!parse_duration(argument, MAX_LOAD_MS, &duration)) {
        record_event(app, "invalid_duration", 0);
        return;
    }
    app->load_ms = duration;
    record_event(app, "load_scheduled", 0);
}

static void enqueue_movement(Application *app, const char *argument, int direction) {
    int duration;
    if (!parse_duration(argument, MAX_MS, &duration)) {
        record_event(app, "invalid_duration", 0);
        return;
    }
    if (app->latched) {
        record_event(app, "rejected_latched", 0);
        return;
    }
    if (!try_acquire(&app->slots)) {
        record_event(app, "rejected_queue_full", 0);
        return;
    }
    Command command = {
        .id = ++app->next_id, .generation = app->generation,
        .direction = direction, .duration_ms = duration,
        .expires_ms = monotonic_ms() + COMMAND_TTL_MS
    };
    app->queue[(app->head + app->count) % QUEUE_CAPACITY] = command;
    ++app->count;
    release_slot(&app->items);
    record_event(app, "command_accepted", command.id);
}

static void handle_forward(Application *app, const char *argument) {
    enqueue_movement(app, argument, 1);
}

static void handle_backward(Application *app, const char *argument) {
    enqueue_movement(app, argument, -1);
}

static void handle_quit(Application *app, const char *argument) {
    (void)argument;
    request_shutdown(app, "quit_requested");
}

static void handle_stop(Application *app, const char *argument) {
    (void)argument;
    request_stop(app, "stop_requested");
}

static void handle_state(Application *app, const char *argument) {
    (void)argument;
    const char *name = app->latched ? "state_latched" : "state_armed";
    record_event(app, name, app->active_id);
}

typedef struct {
    const char *name;
    int field_count;
    void (*handle)(Application *, const char *);
} ConsoleCommand;

static void dispatch_console_command(Application *app, const char *name,
                                     const char *argument, int field_count) {
    static const ConsoleCommand commands[] = {
        {"quit", 1, handle_quit}, {"stop", 1, handle_stop},
        {"arm", 1, handle_arm}, {"state", 1, handle_state},
        {"sensor", 2, handle_sensor}, {"load", 2, handle_load},
        {"forward", 2, handle_forward}, {"backward", 2, handle_backward}
    };
    for (size_t i = 0; i < sizeof commands / sizeof commands[0]; ++i) {
        if (strcmp(name, commands[i].name) != 0) continue;
        if (field_count != commands[i].field_count) break;
        commands[i].handle(app, argument);
        return;
    }
    record_event(app, "invalid_command", 0);
}

static void execute_line(Application *app, char *line) {
    char name[32], argument[64] = "", extra;
    int fields = sscanf(line, "%31s %63s %c", name, argument, &extra);
    if (fields < 1) return;
    pthread_mutex_lock(&app->mutex);
    if (!app->shutting_down) dispatch_console_command(app, name, argument, fields);
    pthread_mutex_unlock(&app->mutex);
}

static bool is_shutting_down(Application *app) {
    pthread_mutex_lock(&app->mutex);
    bool result = app->shutting_down;
    pthread_mutex_unlock(&app->mutex);
    return result;
}

static bool input_poll_failed(int result, short events) {
    if (result < 0) return true;
    if (result == 0) return false;
    return (events & (POLLERR | POLLNVAL)) != 0;
}

static void *input_thread(void *arg) {
    Application *app = arg;
    record_scheduling(app, "scheduling_input");
    char line[128];
    size_t used = 0;
    bool overflowed = false;
    /* Read one byte at a time so stdio cannot hide buffered pipe input from poll.
       The poll timeout lets shutdown proceed without further console input. */
    while (!is_shutting_down(app)) {
        struct pollfd fd = {.fd = STDIN_FILENO, .events = POLLIN};
        int rc = poll(&fd, 1, 50);
        if (rc < 0 && errno == EINTR) continue;
        if (input_poll_failed(rc, fd.revents)) {
            pthread_mutex_lock(&app->mutex);
            app->error = 1;
            request_shutdown(app, "input_error");
            pthread_mutex_unlock(&app->mutex);
            break;
        }
        if (rc == 0) continue;
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) {
            if (n == 0) {
                if (used != 0 && !overflowed) {
                    line[used] = '\0';
                    execute_line(app, line);
                }
            }
            pthread_mutex_lock(&app->mutex);
            if (n < 0) app->error = 1;
            request_shutdown(app, n < 0 ? "input_error" : "input_eof");
            pthread_mutex_unlock(&app->mutex);
            break;
        }
        if (c == '\n') {
            if (!overflowed) {
                line[used] = '\0';
                execute_line(app, line);
            } else {
                pthread_mutex_lock(&app->mutex);
                record_event(app, "line_too_long", 0);
                pthread_mutex_unlock(&app->mutex);
            }
            used = 0;
            overflowed = false;
        } else if (c == '\0') overflowed = true;
        else if (used < sizeof line - 1) line[used++] = c;
        else overflowed = true;
    }
    return NULL;
}

static void wait_next_cycle(Application *app, struct timespec *next, long ms) {
    next->tv_nsec += ms * 1000000L;
    next->tv_sec += next->tv_nsec / 1000000000L;
    next->tv_nsec %= 1000000000L;
    int rc;
    do { rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL); }
    while (rc == EINTR);
    if (rc != 0) {
        pthread_mutex_lock(&app->mutex);
        app->error = 1;
        request_shutdown(app, "timing_error");
        pthread_mutex_unlock(&app->mutex);
    }
}

/* Only the exchange thread calls these functions, with the mutex held.
   Queue operations are nonblocking; this is the only IPC command producer. */
static int send_to_bridge(Application *app, uint32_t type, const Command *command) {
    int64_t now = monotonic_ms();
    Message message = {
        .type = type, .sequence = app->ipc_sequence + 1,
        .generation = app->generation, .sensor = (uint32_t)app->sensor,
        .timestamp_ms = now, .expires_ms = now + RIDER_LEASE_MS
    };
    unsigned priority = 10;
    bool normal = type == C_HEARTBEAT || type == C_MOVE;
    if (type == C_STOP) priority = 30;
    if (type == C_ARM) priority = 20;
    if (type == C_QUIT) priority = 31;
    if (command != NULL) {
        priority = 0;
        message.command_id = command->id;
        message.expires_ms = command->expires_ms;
        message.duration_ms = (uint32_t)command->duration_ms;
        message.direction = command->direction;
    }
    int rc = rider_send(app->bridge.commands, &message, priority,
                         normal ? RIDER_NORMAL_MAX : RIDER_MQ_MAX);
    if (rc == 1) ++app->ipc_sequence;
    if (rc < 0) { app->error = 1; request_shutdown(app, "ipc_send_failed"); }
    return rc;
}

static Event *record_bridge_event(Application *app, const char *name, uint64_t id, const Message *message) {
    Event *e = record_event(app, name, id);
    if (e != NULL) e->bridge_timestamp_ms = message->timestamp_ms;
    return e;
}

static void confirm_stop(Application *app, const Message *message) {
    app->direction = 0;
    app->stop_confirmed = true;
    app->bridge_armed = false;
    app->command_sent = false;
    Event *e = record_bridge_event(app, "stop_confirmed", app->active_id, message);
    if (app->stop_requested_ns != 0) {
        int64_t latency = (monotonic_ns() - app->stop_requested_ns) / 1000;
        if (e != NULL) e->stop_latency_us = latency;
        if (latency > app->max_stop_latency_us) app->max_stop_latency_us = latency;
        app->stop_requested_ns = 0;
    }
    app->active_id = 0;
}

static bool valid_response_state(const Message *message) {
    bool armed = (message->flags & RIDER_ARMED) != 0;
    switch (message->type) {
    case R_STOPPED:
    case R_EXITED:
        return message->direction == 0 && !armed;
    case R_COMPLETED:
        return message->direction == 0;
    case R_ARMED:
        return message->direction == 0 && armed;
    default:
        return true;
    }
}

static bool valid_bridge_response(const Message *message, int64_t now) {
    if (message->type < R_STATUS || message->type > R_EXITED) return false;
    if (message->timestamp_ms > now) return false;
    if (message->sample_ms > now) return false;
    if (message->sample == 0) return false;
    return valid_response_state(message);
}

static bool matches_active_command(const Application *app, const Message *message) {
    if (!app->command_sent) return false;
    return message->command_id == app->active_id;
}

static bool arm_is_pending(const Application *app) {
    if (app->latched) return false;
    return !app->bridge_armed;
}

static bool handle_bridge_response(Application *app, const Message *message, unsigned priority) {
    int64_t now = monotonic_ms();
    if (!valid_bridge_response(message, now)) {
        app->error = 1; request_shutdown(app, "bridge_protocol_fault"); return false;
    }
    unsigned response_class = priority == 0 ? 0 : 1;
    if (message->sequence <= app->received_sequences[response_class] || message->expires_ms <= now) return false;
    app->received_sequences[response_class] = message->sequence;
    app->last_response_ms = now;
    if (message->type == R_FAULT) {
        app->error = 1; request_shutdown(app, "bridge_fault"); return false;
    }
    if (message->generation != app->generation) return false;
    if (message->sample > app->telemetry.sequence) {
        app->telemetry = (Telemetry){
            .sequence = message->sample, .ms = message->sample_ms,
            .valid = (message->flags & RIDER_SAMPLE_VALID) != 0
        };
        record_bridge_event(app, "telemetry", 0, message);
    }
    switch (message->type) {
    case R_STARTED:
        if (app->latched) break;
        if (matches_active_command(app, message)) {
            if (message->direction == 0 || message->end_ms <= message->timestamp_ms) {
                app->error = 1; request_shutdown(app, "bridge_protocol_fault"); break;
            }
            app->direction = message->direction;
            app->movement_end_ms = message->end_ms;
            record_bridge_event(app, "movement_started", message->command_id, message);
        }
        break;
    case R_COMPLETED:
        if (matches_active_command(app, message)) {
            app->direction = 0;
            app->command_sent = false;
            record_bridge_event(app, "movement_completed", message->command_id, message);
            app->active_id = 0;
        }
        break;
    case R_STOPPED:
        if (app->latched && app->stop_sent_generation == app->generation) confirm_stop(app, message);
        break;
    case R_ARMED:
        if (!app->latched && app->arm_sent_generation == app->generation) {
            app->bridge_armed = true;
            record_bridge_event(app, "armed", 0, message);
        }
        break;
    case R_REJECTED:
        if (matches_active_command(app, message)) {
            app->command_sent = false;
            record_bridge_event(app, "command_discarded", message->command_id, message);
            app->active_id = 0;
        } else if (message->command_id == 0 && arm_is_pending(app)) {
            request_stop(app, "arm_rejected_by_bridge");
        }
        break;
    case R_EXITED:
        return app->shutting_down && app->quit_sent;
    default: break;
    }
    return false;
}

static bool read_bridge_responses(Application *app) {
    for (int i = 0; i < RIDER_MQ_MAX; ++i) {
        Message message;
        unsigned priority;
        int rc = rider_receive(app->bridge.responses, &message, &priority);
        if (rc == 0) break;
        if (rc < 0) {
            app->error = 1;
            const char *reason = rc == -2 ? "bridge_protocol_fault" : "ipc_receive_failed";
            request_shutdown(app, reason);
            break;
        }
        if (handle_bridge_response(app, &message, priority)) return true;
    }
    return false;
}

static void send_pending_stop(Application *app, int64_t now) {
    if (!app->latched) return;
    if (app->stop_sent_generation == app->generation) return;
    if (send_to_bridge(app, C_STOP, NULL) != 1) return;
    app->stop_sent_generation = app->generation;
    app->control_ms = now;
}

static void send_pending_arm(Application *app, int64_t now) {
    if (!arm_is_pending(app)) return;
    if (app->arm_sent_generation == app->generation) return;
    if (send_to_bridge(app, C_ARM, NULL) != 1) return;
    app->arm_sent_generation = app->generation;
    app->control_ms = now;
}

static void check_control_timeout(Application *app, int64_t now) {
    if (app->shutting_down) return;
    bool waiting = app->latched ? !app->stop_confirmed : !app->bridge_armed;
    if (!waiting) return;
    if (now - app->control_ms <= RIDER_TIMEOUT_MS) return;
    app->error = 1;
    request_shutdown(app, "bridge_control_unconfirmed");
}

static bool can_dispatch_movement(const Application *app, int64_t now) {
    if (app->shutting_down || app->latched) return false;
    if (!app->bridge_armed || app->command_sent) return false;
    if (app->count == 0) return false;
    return telemetry_health(&app->telemetry, now) == TELEMETRY_HEALTHY;
}

static void dispatch_movements(Application *app, int64_t now) {
    while (can_dispatch_movement(app, now)) {
        Command command = app->queue[app->head];
        bool expired = command.generation != app->generation || now >= command.expires_ms;
        if (!expired && send_to_bridge(app, C_MOVE, &command) != 1) break;
        if (!try_acquire(&app->items)) abort();
        app->head = (app->head + 1) % QUEUE_CAPACITY;
        --app->count;
        release_slot(&app->slots);
        if (expired) {
            record_event(app, "command_discarded", command.id);
            continue;
        }
        app->command_sent = true;
        app->active_id = command.id;
        record_event(app, "command_sent", command.id);
    }
}

static void send_pending_quit(Application *app) {
    if (!app->shutting_down) return;
    if (!app->stop_confirmed || app->quit_sent) return;
    if (send_to_bridge(app, C_QUIT, NULL) == 1) app->quit_sent = true;
}

static bool exchange_cycle(Application *app, int64_t *shutdown_started_ms) {
    int64_t now = monotonic_ms();
    bool quit = read_bridge_responses(app);
    int child = bridge_poll(&app->bridge);
    if (quit) return true;
    if (child != 0) {
        app->error = 1;
        request_shutdown(app, "bridge_exited_unexpectedly");
        return true;
    }
    if (now - app->last_response_ms > RIDER_TIMEOUT_MS) {
        app->error = 1;
        request_shutdown(app, "bridge_unresponsive");
    }
    if (app->shutting_down && *shutdown_started_ms == 0) *shutdown_started_ms = now;
    send_pending_stop(app, now);
    if (now - app->last_heartbeat_ms >= 100) {
        if (send_to_bridge(app, C_HEARTBEAT, NULL) == 1) app->last_heartbeat_ms = now;
    }
    send_pending_arm(app, now);
    check_control_timeout(app, now);
    dispatch_movements(app, now);
    send_pending_quit(app);
    if (*shutdown_started_ms == 0) return false;
    if (now - *shutdown_started_ms < 500) return false;
    app->error = 1;
    record_event(app, "bridge_shutdown_unconfirmed", 0);
    return true;
}

static void *exchange_thread(void *arg) {
    Application *app = arg;
    record_scheduling(app, "scheduling_exchange");
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int64_t shutdown_started_ms = 0;
    for (;;) {
        pthread_mutex_lock(&app->mutex);
        bool quit = exchange_cycle(app, &shutdown_started_ms);
        pthread_mutex_unlock(&app->mutex);
        if (quit) break;
        wait_next_cycle(app, &next, 10);
    }
    return NULL;
}

static bool is_timer_notification(const Application *app, int signal_number,
                                  const siginfo_t *info) {
    if (signal_number != SIGRTMIN) return false;
    if (info->si_code != SI_TIMER) return false;
    return info->si_value.sival_ptr == app;
}

static bool movement_deadline_missed(const Application *app) {
    if (app->latched || app->direction == 0) return false;
    return monotonic_ms() > app->movement_end_ms + 100;
}

static void *supervision_thread(void *arg) {
    Application *app = arg;
    record_scheduling(app, "scheduling_supervision");
    int64_t previous_ns = 0;
    const int64_t period_ns = SUPERVISION_MS * 1000000LL;
    while (!is_shutting_down(app)) {
        siginfo_t info;
        struct timespec timeout = {.tv_sec = 0, .tv_nsec = 100000000};
        int signal_number = sigtimedwait(&app->signals, &info, &timeout);
        bool retry_wait = errno == EINTR || errno == EAGAIN;
        if (signal_number < 0 && retry_wait) continue;
        pthread_mutex_lock(&app->mutex);
        if (signal_number < 0) {
            app->error = 1;
            request_shutdown(app, "signal_wait_failed");
        } else if (signal_number == SIGINT || signal_number == SIGTERM) request_shutdown(app, "signal_received");
        if (app->shutting_down) { pthread_mutex_unlock(&app->mutex); break; }
        if (!is_timer_notification(app, signal_number, &info)) {
            pthread_mutex_unlock(&app->mutex);
            continue;
        }
        int load = app->load_ms;
        app->load_ms = 0;
        pthread_mutex_unlock(&app->mutex);
        /* Inject bounded work in this thread, outside the critical section. */
        if (load > 0) {
            int64_t until_ns = monotonic_ns() + load * 1000000LL;
            while (monotonic_ns() < until_ns) { /* Simulated work. */ }
        }
        int64_t timestamp_ns = monotonic_ns();
        int overruns = timer_getoverrun(app->timer);
        if (overruns < 0) {
            pthread_mutex_lock(&app->mutex);
            app->error = 1;
            request_shutdown(app, "timer_overrun_failed");
            pthread_mutex_unlock(&app->mutex);
            break;
        }
        /* Overruns belong to the consumed notification, not to a notification
           that became pending during the injected load. */
        int64_t expirations = (int64_t)overruns + 1;
        pthread_mutex_lock(&app->mutex);
        Telemetry snapshot = app->telemetry;
        pthread_mutex_unlock(&app->mutex);
        int fault = telemetry_health(&snapshot, timestamp_ns / 1000000);
        struct timespec cpu;
        int64_t cpu_us = clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu) == 0
            ? (int64_t)cpu.tv_sec * 1000000 + cpu.tv_nsec / 1000 : -1;
        pthread_mutex_lock(&app->mutex);
        int64_t lateness = (timestamp_ns - app->next_supervision_ns) / 1000;
        if (lateness < 0) lateness = 0;
        ++app->cycles;
        app->missed_expirations += (uint64_t)overruns;
        if (lateness >= SUPERVISION_MS * 1000) ++app->late_cycles;
        if (lateness > app->max_lateness_us) app->max_lateness_us = lateness;
        Event *e = record_event(app, "supervision_cycle", 0);
        if (e != NULL) {
            e->lateness_us = lateness;
            e->overruns = overruns;
            e->thread_cpu_us = cpu_us;
            if (previous_ns != 0) {
                e->interval_us = (timestamp_ns - previous_ns) / 1000;
                e->jitter_us = (timestamp_ns - previous_ns - expirations * period_ns) / 1000;
            }
        }
        previous_ns = timestamp_ns;
        app->next_supervision_ns += expirations * period_ns;
        /* A replacement sample is evaluated on the next cycle; an outdated
           snapshot must not latch the controller. */
        if (snapshot.sequence == app->telemetry.sequence && fault != app->telemetry_fault) {
            app->telemetry_fault = fault;
            if (fault == TELEMETRY_HEALTHY) record_event(app, "telemetry_recovered", 0);
            else request_stop(app, fault == TELEMETRY_INVALID ? "invalid_telemetry_fault" : "stale_telemetry_fault");
        }
        if (movement_deadline_missed(app))
            request_stop(app, "movement_deadline_fault");
        pthread_mutex_unlock(&app->mutex);
    }
    return NULL;
}

static bool write_event_csv(FILE *csv, const Event *entry) {
    int written = fprintf(csv,
        "%" PRId64 ",%s,%" PRIu64 ",%" PRIu64 ",%zu,%d,"
        "%" PRIu64 ",%" PRId64 ",%d,%" PRId64 ",%d,%" PRId64
        ",%" PRId64 ",%" PRId64 ",%d,%d,%" PRId64 ",%" PRId64 "\n",
        entry->ms, entry->name, entry->id, entry->generation,
        entry->pending, entry->direction, entry->sample, entry->sample_age_ms,
        entry->valid, entry->lateness_us, entry->overruns, entry->interval_us,
        entry->jitter_us, entry->stop_latency_us, entry->policy, entry->priority,
        entry->thread_cpu_us, entry->bridge_timestamp_ms);
    if (written < 0) return false;
    return fflush(csv) == 0;
}

static bool write_csv_header(FILE *csv) {
    const char *header =
        "ms,event,id,generation,pending,direction,sample,sample_age_ms,valid,"
        "lateness_us,missed_expirations,interval_us,jitter_us,stop_latency_us,"
        "policy,priority,thread_cpu_us,bridge_timestamp_ms\n";
    if (fputs(header, csv) == EOF) return false;
    return fflush(csv) == 0;
}

static void *logger_thread(void *arg) {
    Application *app = arg;
    record_scheduling(app, "scheduling_logger");
    bool fault = false;
    for (;;) {
        pthread_mutex_lock(&app->mutex);
        while (app->event_count == 0 && !app->logger_done)
            pthread_cond_wait(&app->event_available, &app->mutex);
        if (app->event_count == 0 && app->logger_done) {
            pthread_mutex_unlock(&app->mutex);
            break;
        }
        Event e = app->events[app->event_head];
        app->event_head = (app->event_head + 1) % EVENT_CAPACITY;
        --app->event_count;
        pthread_mutex_unlock(&app->mutex);
        if (!write_event_csv(app->csv, &e)) fault = true;
        if (strcmp(e.name, "supervision_cycle") != 0 && strcmp(e.name, "telemetry") != 0)
            printf("[%" PRId64 " ms] %s id=%" PRIu64 " generation=%" PRIu64
               " pending=%zu direction=%d\n", e.ms, e.name, e.id,
               e.generation, e.pending, e.direction);
        if (fault) {
            pthread_mutex_lock(&app->mutex);
            app->error = 1;
            request_shutdown(app, "log_write_failed");
            pthread_mutex_unlock(&app->mutex);
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    if (argc > 3) {
        fprintf(stderr, "Usage: %s [events.csv [bridge_executable]]\n", argv[0]);
        return 1;
    }
    Application app = {
        .mutex = PTHREAD_MUTEX_INITIALIZER,
        .event_available = PTHREAD_COND_INITIALIZER,
        .latched = true, .generation = 1,
        .bridge = {.commands = (mqd_t)-1, .responses = (mqd_t)-1, .pid = -1}
    };
    app.start_ms = monotonic_ms();
    bool timer_created = false;
    bool bridge_created = false;
    app.csv = fopen(argc >= 2 ? argv[1] : "rider_sim.csv", "w");
    if (app.csv == NULL) { perror("CSV log"); return 1; }
    if (!write_csv_header(app.csv)) {
        perror("CSV header");
        fclose(app.csv);
        return 1;
    }
    if (sem_init(&app.slots, 0, QUEUE_CAPACITY) != 0) {
        perror("sem_init slots"); fclose(app.csv); return 1;
    }
    if (sem_init(&app.items, 0, 0) != 0) {
        perror("sem_init items"); sem_destroy(&app.slots); fclose(app.csv); return 1;
    }
    /* All threads inherit this mask. Only the supervisor consumes these
       signals synchronously; no asynchronous signal handlers are installed. */
    sigemptyset(&app.signals);
    sigaddset(&app.signals, SIGINT);
    sigaddset(&app.signals, SIGTERM);
    sigaddset(&app.signals, SIGRTMIN);
    int mask_result = pthread_sigmask(SIG_BLOCK, &app.signals, NULL);
    if (mask_result != 0) {
        fprintf(stderr, "pthread_sigmask: %s\n", strerror(mask_result));
        app.error = 1;
        goto cleanup;
    }
    char bridge_path[4096];
    const char *bridge_executable = argc == 3 ? argv[2] : bridge_path;
    if (argc != 3) {
        /* Resolve the bridge beside this executable, regardless of the working directory. */
        ssize_t n = readlink("/proc/self/exe", bridge_path, sizeof bridge_path - 1);
        if (n < 0 || (size_t)n >= sizeof bridge_path - 1) {
            perror("executable path"); app.error = 1; goto cleanup;
        }
        bridge_path[n] = '\0';
        char *slash = strrchr(bridge_path, '/');
        size_t available = sizeof bridge_path - (size_t)(slash + 1 - bridge_path);
        if (snprintf(slash + 1, available, "rider_bridge_sim") >= (int)available) {
            fprintf(stderr, "Bridge path is too long\n"); app.error = 1; goto cleanup;
        }
    }
    Message ready;
    if (bridge_start(&app.bridge, bridge_executable, &ready) != 0) {
        perror("bridge startup"); app.error = 1; goto cleanup;
    }
    bridge_created = true;
    app.telemetry = (Telemetry){
        .sequence = ready.sample, .ms = ready.sample_ms,
        .valid = (ready.flags & RIDER_SAMPLE_VALID) != 0
    };
    app.stop_confirmed = true;
    app.stop_sent_generation = app.generation;
    app.received_sequences[1] = ready.sequence;
    app.last_response_ms = app.control_ms = monotonic_ms();
    record_event(&app, "bridge_started", (uint64_t)app.bridge.pid);
    struct sigevent notification = {
        .sigev_notify = SIGEV_SIGNAL, .sigev_signo = SIGRTMIN,
        .sigev_value = {.sival_ptr = &app}
    };
    if (timer_create(CLOCK_MONOTONIC, &notification, &app.timer) != 0) {
        perror("timer_create"); app.error = 1; goto cleanup;
    }
    timer_created = true;
    app.next_supervision_ns = monotonic_ns() + SUPERVISION_MS * 1000000LL;
    struct itimerspec period = {
        .it_interval = {.tv_sec = 0, .tv_nsec = SUPERVISION_MS * 1000000L},
        .it_value = {
            .tv_sec = (time_t)(app.next_supervision_ns / 1000000000),
            .tv_nsec = (long)(app.next_supervision_ns % 1000000000)
        }
    };
    if (timer_settime(app.timer, TIMER_ABSTIME, &period, NULL) != 0) {
        perror("timer_settime"); app.error = 1; goto cleanup;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    puts("SIMULATED Rider. Initially stopped and latched. No robot connection.");
    printf("Simulated bridge: pid=%ld commands=%s responses=%s\n", (long)app.bridge.pid,
           app.bridge.command_queue_name, app.bridge.response_queue_name);
    puts("Commands: arm | forward MS | backward MS | stop | state | quit");
    puts("Simulation tests: sensor normal|invalid|frozen | load MS (1..200)");
    puts("Duration: 1..3000 ms; queue expiry: 2000 ms; Ctrl+C or EOF shuts down.");
    pthread_t threads[4];
    void *(*tasks[4])(void *) = {logger_thread, exchange_thread, supervision_thread, input_thread};
    size_t created = 0;
    for (; created < 4; ++created) {
        int rc = pthread_create(&threads[created], NULL, tasks[created], &app);
        if (rc != 0) {
            fprintf(stderr, "pthread_create: %s\n", strerror(rc));
            pthread_mutex_lock(&app.mutex);
            app.error = 1;
            request_shutdown(&app, "thread_creation_failed");
            pthread_mutex_unlock(&app.mutex);
            break;
        }
    }
    /* Join the logger last so it can drain events from all other threads. */
    for (size_t i = 1; i < created; ++i) pthread_join(threads[i], NULL);
    int bridge_result = bridge_finish(&app.bridge);
    bridge_created = false;
    pthread_mutex_lock(&app.mutex);
    if (bridge_result != 0) app.error = 1;
    record_event(&app, bridge_result == 0 ? "bridge_reaped" : "bridge_reaped_with_error", 0);
    record_event(&app, "shutdown_completed", 0);
    app.logger_done = true;
    pthread_cond_signal(&app.event_available);
    pthread_mutex_unlock(&app.mutex);
    if (created > 0) pthread_join(threads[0], NULL);
    printf("Metrics: cycles=%" PRIu64 " missed_expirations=%" PRIu64
           " late_cycles=%" PRIu64 " max_lateness_us=%" PRId64
           " max_stop_latency_us=%" PRId64 ".\n",
           app.cycles, app.missed_expirations, app.late_cycles,
           app.max_lateness_us, app.max_stop_latency_us);
    printf("Done: %zu threads joined; dropped events=%lu.\n", created, app.dropped_events);
cleanup:
    if (bridge_created && bridge_finish(&app.bridge) != 0) app.error = 1;
    if (timer_created && timer_delete(app.timer) != 0) {
        perror("timer_delete"); app.error = 1;
    }
    sem_destroy(&app.items);
    sem_destroy(&app.slots);
    pthread_cond_destroy(&app.event_available);
    pthread_mutex_destroy(&app.mutex);
    if (fclose(app.csv) != 0) { perror("CSV close"); app.error = 1; }
    return app.error ? 1 : 0;
}
