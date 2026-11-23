#ifndef XGO_C_IPC_BRIDGE_H
#define XGO_C_IPC_BRIDGE_H

#include <mqueue.h>
#include <stdbool.h>

typedef struct {
    mqd_t commands;
    mqd_t telemetry;
} IpcBridge;

int ipc_bridge_open(IpcBridge *bridge);
void *ipc_worker(void *argument);
void ipc_bridge_close(IpcBridge *bridge);

#endif
