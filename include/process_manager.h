#ifndef XGO_PROCESS_MANAGER_H
#define XGO_PROCESS_MANAGER_H

#include <stdbool.h>
#include <sys/types.h>

typedef struct {
    pid_t pid;
    int status;
} BridgeProcess;

int process_start(BridgeProcess *child, const char *python, bool mock, bool telemetry_only);
int process_poll(BridgeProcess *child);
int process_stop(BridgeProcess *child);

#endif
