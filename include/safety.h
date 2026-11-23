#ifndef XGO_SAFETY_H
#define XGO_SAFETY_H

#include "shared_state.h"

bool safety_bridge_ready(const RobotStateSnapshot *state, uint64_t now);
bool safety_motion_allowed(const RobotStateSnapshot *state, uint64_t now);
bool safety_reset_allowed(const RobotStateSnapshot *state, uint64_t now);
void safety_latch_fault(RobotStateSnapshot *state, FaultCode fault, uint64_t now);
void safety_evaluate(RobotStateSnapshot *state, uint64_t now);
const char *safety_state_name(SafetyState state);
const char *fault_name(FaultCode fault);
void *safety_worker(void *argument);

#endif
