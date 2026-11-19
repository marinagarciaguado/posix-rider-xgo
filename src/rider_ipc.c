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
static void sleep_briefly(void) {
    struct timespec t = {.tv_nsec = 10000000};
    while (nanosleep(&t, &t) != 0 && errno == EINTR) { }
}
int rider_send(mqd_t queue, const Message *m, unsigned priority, long limit) {
    struct mq_attr attr;
    if (mq_getattr(queue, &attr) != 0) return -1;
    if (attr.mq_curmsgs >= limit) return 0;
    unsigned char data[RIDER_WIRE_SIZE];
    rider_encode(m, data);
    int rc;
    do { rc = mq_send(queue, (const char *)data, sizeof data, priority); }
    while (rc != 0 && errno == EINTR);
    if (rc == 0) return 1;
    return errno == EAGAIN ? 0 : -1;
}
int rider_receive(mqd_t queue, Message *m, unsigned *priority) {
    unsigned char data[RIDER_WIRE_SIZE];
    ssize_t n;
    do { n = mq_receive(queue, (char *)data, sizeof data, priority); }
    while (n < 0 && errno == EINTR);
    if (n < 0) return errno == EAGAIN ? 0 : -1;
    return rider_decode(data, (size_t)n, m) ? 1 : -2;
}
int bridge_poll(Bridge *p) {
    if (p->reaped || p->pid <= 0) return 1;
    pid_t rc;
    do { rc = waitpid(p->pid, &p->state, WNOHANG); } while (rc < 0 && errno == EINTR);
    if (rc == p->pid) { p->reaped = true; return 1; }
    return rc < 0 ? -1 : 0;
}
static bool wait_for_child(Bridge *p, int timeout_ms) {
    int64_t end = ms() + timeout_ms;
    while (ms() < end) {
        int rc = bridge_poll(p);
        if (rc != 0) return rc == 1;
        sleep_briefly();
    }
    return bridge_poll(p) == 1;
}
static bool child_exited_successfully(const Bridge *bridge) {
    if (!bridge->reaped) return false;
    if (!WIFEXITED(bridge->state)) return false;
    return WEXITSTATUS(bridge->state) == 0;
}

int bridge_finish(Bridge *p) {
    int result = 0;
    if (p->pid > 0) {
        if (!wait_for_child(p, 250)) {
            result = -1;
            /* SIGCONT also allows cleanup of a simulator suspended with SIGSTOP. */
            kill(p->pid, SIGCONT);
            kill(p->pid, SIGTERM);
            if (!wait_for_child(p, 250)) {
                kill(p->pid, SIGKILL);
                pid_t rc;
                do { rc = waitpid(p->pid, &p->state, 0); } while (rc < 0 && errno == EINTR);
                p->reaped = rc == p->pid;
            }
        }
        if (!child_exited_successfully(p)) result = -1;
    }
    if (p->commands != (mqd_t)-1 && mq_close(p->commands) != 0) result = -1;
    if (p->responses != (mqd_t)-1 && mq_close(p->responses) != 0) result = -1;
    p->commands = p->responses = (mqd_t)-1;
    if (p->command_queue_name[0] && mq_unlink(p->command_queue_name) != 0) result = -1;
    if (p->response_queue_name[0] && mq_unlink(p->response_queue_name) != 0) result = -1;
    p->command_queue_name[0] = p->response_queue_name[0] = '\0';
    return result;
}
static bool valid_ready_message(const Message *message) {
    if (message->type != R_READY) return false;
    if (message->generation != 1) return false;
    if (message->direction != 0) return false;
    if (message->flags != RIDER_SAMPLE_VALID) return false;
    if (message->sample == 0) return false;
    int64_t now = ms();
    if (message->sample_ms > now) return false;
    return message->expires_ms > now;
}

int bridge_start(Bridge *p, const char *path, Message *ready) {
    *p = (Bridge){.commands = (mqd_t)-1, .responses = (mqd_t)-1, .pid = -1};
    char commands[96], responses[96];
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) return -1;
    snprintf(commands, sizeof commands, "/rider_%ld_%ld_o", (long)getpid(), t.tv_nsec);
    snprintf(responses, sizeof responses, "/rider_%ld_%ld_r", (long)getpid(), t.tv_nsec);
    struct mq_attr attr = {.mq_maxmsg = RIDER_MQ_MAX, .mq_msgsize = RIDER_WIRE_SIZE};
    p->commands = mq_open(commands, O_CREAT | O_EXCL | O_RDWR | O_NONBLOCK, 0600, &attr);
    if (p->commands == (mqd_t)-1) return -1;
    strcpy(p->command_queue_name, commands); /* Only unlink queues created by this process. */
    p->responses = mq_open(responses, O_CREAT | O_EXCL | O_RDWR | O_NONBLOCK, 0600, &attr);
    if (p->responses == (mqd_t)-1) goto fault;
    strcpy(p->response_queue_name, responses);
    p->pid = fork(); /* Fork before creating any supervisor threads. */
    if (p->pid < 0) goto fault;
    if (p->pid == 0) {
        execl(path, path, commands, responses, (char *)NULL);
        _exit(127);
    }
    int64_t end = ms() + 1500;
    while (ms() < end) {
        unsigned priority;
        int rc = rider_receive(p->responses, ready, &priority);
        if (rc == 1 && valid_ready_message(ready)) return 0;
        if (rc != 0 || bridge_poll(p) != 0) { errno = EPROTO; goto fault; }
        sleep_briefly();
    }
    errno = ETIMEDOUT;
fault: {
        int error = errno;
        bridge_finish(p);
        errno = error;
        return -1;
    }
}
