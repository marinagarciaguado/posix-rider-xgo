#ifndef RIDER_IPC_H
#define RIDER_IPC_H

#include <mqueue.h>
#include <sys/types.h>
#include "rider_protocol.h"

typedef struct {
    mqd_t commands, responses;
    pid_t pid;
    int state;
    bool reaped;
    char command_queue_name[96], response_queue_name[96];
} Bridge;

/* Return 1 for success, 0 when unavailable, -1 on error, -2 for malformed data. */
int rider_send(mqd_t queue, const Message *m, unsigned priority, long limit);
int rider_receive(mqd_t queue, Message *m, unsigned *priority);
int bridge_start(Bridge *p, const char *path, Message *ready);
/* Return 1 if reaped, 0 if running, -1 on error. One caller during operation. */
int bridge_poll(Bridge *p);
/* Always close and unlink owned queues; terminate and reap the child if needed.
   Return -1 for forced termination, abnormal exit, or cleanup failure. */
int bridge_finish(Bridge *p);

#endif
