#define _POSIX_C_SOURCE 200809L
#include "rider_ipc.h"
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Owns the simulated model. No SDK calls or robot access. */
typedef struct {
    mqd_t commands, responses;
    pid_t parent_pid;
    uint64_t sequence, generation, command, sample, received_sequences[6], stop_sequence;
    int direction, sensor;
    bool armed, valid, channel_error, watchdog;
    int64_t start_ms, end_ms, sample_ms, next_sample_ms, lease_ms, next_status_ms;
} Model;

static int64_t ms(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) _exit(2);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}
static int send_response(Model *model, uint32_t type, uint32_t reason, uint64_t command) {
    int64_t now = ms();
    Message message = {
        .type = type, .flags = (model->armed ? RIDER_ARMED : 0) |
                             (model->valid ? RIDER_SAMPLE_VALID : 0),
        .sequence = model->sequence + 1, .generation = model->generation,
        .command_id = command, .expires_ms = now + RIDER_TIMEOUT_MS,
        .direction = model->direction, .sensor = (uint32_t)model->sensor, .reason = reason,
        .sample = model->sample, .sample_ms = model->sample_ms,
        .timestamp_ms = type == R_STARTED ? model->start_ms : now, .end_ms = model->end_ms
    };
    bool normal = type == R_STATUS;
    int rc = rider_send(model->responses, &message, normal ? 0 : 20,
                         normal ? RIDER_RESP_NORMAL_MAX : RIDER_MQ_MAX);
    if (rc == 1) ++model->sequence;
    if (rc < 0 || (rc == 0 && !normal)) {
        model->channel_error = true;
        model->armed = false;
        model->direction = 0;
    }
    return rc;
}
static void stop_model(Model *model) {
    model->armed = false;
    model->direction = 0;
    model->command = 0;
    model->end_ms = 0;
}
static void reject_command(Model *model, const Message *message, uint32_t reason) {
    send_response(model, R_REJECTED, reason, message->command_id);
}
static bool sample_is_healthy(const Model *model, int64_t now) {
    if (!model->valid) return false;
    return now - model->sample_ms <= RIDER_MAX_SAMPLE_AGE_MS;
}

static bool lease_is_active(const Model *model, int64_t now) {
    if (model->watchdog) return false;
    return now < model->lease_ms;
}

static bool model_can_arm(const Model *model, int64_t now) {
    if (model->direction != 0) return false;
    if (model->channel_error) return false;
    if (!lease_is_active(model, now)) return false;
    if (model->sensor != 0) return false;
    return sample_is_healthy(model, now);
}

static bool generation_is_allowed(const Model *model, const Message *message) {
    if (message->generation < model->generation) return false;
    if (message->generation == model->generation) return true;
    return message->type == C_STOP || message->type == C_QUIT;
}

static bool command_precedes_stop(const Model *model, const Message *message) {
    if (message->sequence > model->stop_sequence) return false;
    return message->type == C_ARM || message->type == C_MOVE;
}

static uint32_t movement_rejection(const Model *model, const Message *message, int64_t now) {
    if (!model->armed) return REASON_NOT_ARMED;
    if (!lease_is_active(model, now)) return REASON_NOT_ARMED;
    if (model->direction != 0) return REASON_BUSY;
    if (!sample_is_healthy(model, now)) return REASON_SENSOR;
    if (message->command_id == 0) return REASON_PROTOCOL;
    if (message->duration_ms < 1) return REASON_PROTOCOL;
    if (message->duration_ms > RIDER_MAX_MS) return REASON_PROTOCOL;
    if (message->direction == 0) return REASON_PROTOCOL;
    return REASON_NONE;
}

static void start_movement(Model *model, const Message *message, int64_t now) {
    uint32_t rejection = movement_rejection(model, message, now);
    if (rejection != REASON_NONE) {
        reject_command(model, message, rejection);
        return;
    }
    model->direction = message->direction;
    model->command = message->command_id;
    model->start_ms = now;
    model->end_ms = now + message->duration_ms;
    send_response(model, R_STARTED, REASON_NONE, model->command);
}

static bool process_command(Model *model, const Message *message) {
    int64_t now = ms();
    if (!rider_is_command(message->type)) {
        stop_model(model);
        send_response(model, R_FAULT, REASON_PROTOCOL, 0);
        return false;
    }
    /* Track sequences per command type: control priority may overtake movement
       without making that movement a replay. */
    if (message->sequence <= model->received_sequences[message->type]) {
        reject_command(model, message, REASON_PROTOCOL);
        return false;
    }
    model->received_sequences[message->type] = message->sequence;
    if (message->expires_ms <= now || message->timestamp_ms > now) {
        reject_command(model, message, REASON_EXPIRED);
        return false;
    }
    if (!generation_is_allowed(model, message)) {
        reject_command(model, message, REASON_GENERATION);
        return false;
    }
    if (command_precedes_stop(model, message)) {
        reject_command(model, message, REASON_PROTOCOL);
        return false;
    }
    switch (message->type) {
    case C_STOP:
        model->generation = message->generation;
        model->stop_sequence = message->sequence;
        stop_model(model);
        send_response(model, R_STOPPED, REASON_NONE, 0);
        break;
    case C_QUIT:
        model->generation = message->generation;
        stop_model(model);
        send_response(model, R_EXITED, REASON_NONE, 0);
        return true;
    case C_HEARTBEAT:
        if (message->expires_ms > now + RIDER_LEASE_MS) {
            reject_command(model, message, REASON_PROTOCOL);
            break;
        }
        model->lease_ms = message->expires_ms;
        model->watchdog = false;
        model->sensor = (int)message->sensor;
        break;
    case C_ARM:
        if (!model_can_arm(model, now)) {
            reject_command(model, message, REASON_SENSOR);
            break;
        }
        model->armed = true;
        send_response(model, R_ARMED, REASON_NONE, 0);
        break;
    case C_MOVE:
        start_movement(model, message, now);
        break;
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc != 3) { fprintf(stderr, "Usage: %s COMMAND_QUEUE RESPONSE_QUEUE\n", argv[0]); return 2; }
    /* Ctrl+C belongs to the supervisor. SIGTERM remains available when
       the child does not respond to the shutdown protocol. */
    struct sigaction ignore = {.sa_handler = SIG_IGN};
    sigemptyset(&ignore.sa_mask);
    sigset_t empty_set;
    sigemptyset(&empty_set);
    if (sigaction(SIGINT, &ignore, NULL) != 0 || sigprocmask(SIG_SETMASK, &empty_set, NULL) != 0)
        return 2;
    Model model = {
        .commands = mq_open(argv[1], O_RDONLY | O_NONBLOCK),
        .responses = mq_open(argv[2], O_WRONLY | O_NONBLOCK),
        .parent_pid = getppid(), .generation = 1, .sample = 1, .valid = true
    };
    if (model.commands == (mqd_t)-1 || model.responses == (mqd_t)-1) {
        perror("mq_open bridge");
        if (model.commands != (mqd_t)-1) mq_close(model.commands);
        if (model.responses != (mqd_t)-1) mq_close(model.responses);
        return 2;
    }
    model.sample_ms = ms();
    model.next_sample_ms = model.sample_ms + RIDER_SAMPLE_PERIOD_MS;
    model.lease_ms = model.sample_ms + RIDER_LEASE_MS;
    send_response(&model, R_READY, REASON_NONE, 0);
    bool quit = false;
    int result = 0;
    while (!quit) {
        int64_t now = ms();
        if (getppid() != model.parent_pid) { stop_model(&model); result = 2; break; }
        if (now >= model.lease_ms && !model.watchdog) {
            model.watchdog = true;
            stop_model(&model);
            send_response(&model, R_FAULT, REASON_WATCHDOG, 0);
        }
        if (model.channel_error) {
            stop_model(&model);
            /* Retry one fault notification without blocking the actuator. */
            if (send_response(&model, R_FAULT, REASON_SATURATION, 0) == 1) model.channel_error = false;
        }
        if (now >= model.next_sample_ms) {
            if (model.sensor != 2) {
                ++model.sample;
                model.sample_ms = now;
                model.valid = model.sensor == 0;
            }
            model.next_sample_ms = now + RIDER_SAMPLE_PERIOD_MS;
        }
        if (model.direction != 0 && now >= model.end_ms) {
            uint64_t id = model.command;
            model.direction = 0;
            model.command = 0;
            send_response(&model, R_COMPLETED, REASON_NONE, id);
        }
        for (int i = 0; i < RIDER_MQ_MAX && !quit; ++i) {
            Message message;
            unsigned priority;
            int rc = rider_receive(model.commands, &message, &priority);
            if (rc == 0) break;
            if (rc < 0) {
                stop_model(&model);
                send_response(&model, R_FAULT, REASON_PROTOCOL, 0);
                if (rc == -1) { result = 2; quit = true; }
                break;
            }
            quit = process_command(&model, &message);
        }
        if (!quit && now >= model.next_status_ms) {
            send_response(&model, R_STATUS, REASON_NONE, model.command);
            model.next_status_ms = now + 100;
        }
        if (!quit) {
            struct timespec t = {.tv_nsec = 10000000};
            while (nanosleep(&t, &t) != 0 && errno == EINTR) { }
        }
    }
    stop_model(&model);
    mq_close(model.commands);
    mq_close(model.responses);
    return result;
}
