#ifndef XGO_CMD_QUEUE_H
#define XGO_CMD_QUEUE_H

#include "ipc_protocol.h"

typedef struct {
    XgoCmdMsg message;
    uint64_t generation;
} QueuedCommand;

int cmd_queue_init(void);
bool cmd_queue_push(const QueuedCommand *command);
bool cmd_queue_pop(QueuedCommand *command);
void cmd_queue_discard_before(uint64_t generation);
void cmd_queue_destroy(void);

#endif
