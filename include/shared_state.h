#ifndef XGO_SHARED_STATE_H
#define XGO_SHARED_STATE_H

#include "ipc_protocol.h"

typedef enum {
    SAFETY_STARTING, SAFETY_NORMAL, SAFETY_EMERGENCY,
    SAFETY_RECOVERY, SAFETY_SHUTDOWN
} SafetyState;

typedef enum {
    FAULT_NONE, FAULT_TILT, FAULT_OPERATOR_TIMEOUT, FAULT_TELEMETRY_TIMEOUT,
    FAULT_INVALID_TELEMETRY, FAULT_BRIDGE, FAULT_LOW_BATTERY,
    FAULT_OPERATOR_ESTOP, FAULT_RECOVERY_TIMEOUT
} FaultCode;

typedef struct {
    XgoTelemMsg telemetry;
    uint64_t telemetry_rx_ns;
    uint64_t operator_rx_ns;
    uint64_t fault_ns;
    uint64_t estop_sent_ns;
    uint64_t reset_sent_ns;
    uint64_t recovery_started_ns;
    uint64_t generation;
    SafetyState safety;
    FaultCode fault;
    bool child_alive;
    bool bridge_initialized;
    bool runtime_failed;
    bool reset_requested;
} RobotStateSnapshot;

int shared_state_init(void);
RobotStateSnapshot shared_state_snapshot(void);
void shared_state_telemetry(const XgoTelemMsg *msg, uint64_t now);
void shared_state_operator_seen(uint64_t now);
void shared_state_child_exited(void);
void shared_state_fail(void);
void shared_state_shutdown(void);
void shared_state_estop(FaultCode fault, uint64_t now);
void shared_state_request_reset(void);
void shared_state_note_estop(uint64_t generation, uint64_t now);
void shared_state_note_reset(uint64_t generation, uint64_t now);
void shared_state_evaluate(void);
void shared_state_destroy(void);

#endif
