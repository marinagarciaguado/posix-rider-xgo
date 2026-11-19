#define _POSIX_C_SOURCE 200809L
#include "rider_ipc.h"
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

int main(void) {
    char name[80];
    snprintf(name, sizeof name, "/rider_test_%ld", (long)getpid());
    struct mq_attr attr = {.mq_maxmsg = RIDER_MQ_MAX, .mq_msgsize = RIDER_WIRE_SIZE};
    mqd_t q = mq_open(name, O_CREAT | O_EXCL | O_RDWR | O_NONBLOCK, 0600, &attr);
    assert(q != (mqd_t)-1);
    /* Unlink immediately so failed assertions cannot leave a named queue. */
    assert(mq_unlink(name) == 0);
    Message m = {.type = C_MOVE, .sequence = 1, .generation = 1};
    for (int i = 0; i < RIDER_NORMAL_MAX; ++i)
        assert(rider_send(q, &m, 0, RIDER_NORMAL_MAX) == 1);
    assert(rider_send(q, &m, 0, RIDER_NORMAL_MAX) == 0);
    m.type = C_STOP;
    m.generation = 2;
    assert(rider_send(q, &m, 30, RIDER_MQ_MAX) == 1);
    Message received;
    unsigned priority;
    assert(rider_receive(q, &received, &priority) == 1);
    assert(received.type == C_STOP);
    assert(priority == 30);
    assert(received.generation == 2);
    for (int i = 0; i < RIDER_NORMAL_MAX; ++i) {
        assert(rider_receive(q, &received, &priority) == 1);
        assert(received.type == C_MOVE && priority == 0);
    }
    assert(rider_receive(q, &received, &priority) == 0);
    mq_close(q);
    puts("IPC: reserved capacity, stop priority, and nonblocking receive OK");
    return 0;
}
