#include "safety.h"

#include "robot_config.h"
#include "scheduler_utils.h"

#include <math.h>

static bool safety_telemetry_current(const RobotStateSnapshot *state, uint64_t now)
{
    return timestamp_fresh(now, state->telemetry_rx_ns, TELEMETRY_TIMEOUT_MS) &&
           timestamp_fresh(now, state->telemetry.timestamp_ns, TELEMETRY_TIMEOUT_MS) &&
           (state->telemetry.status_flags & XGO_STATUS_SENSOR_VALID);
}

static FaultCode safety_fault(const RobotStateSnapshot *state, uint64_t now)
{
    if (!state->child_alive)
        return FAULT_BRIDGE;
    if (!timestamp_fresh(now, state->telemetry_rx_ns, TELEMETRY_TIMEOUT_MS) ||
        !timestamp_fresh(now, state->telemetry.timestamp_ns, TELEMETRY_TIMEOUT_MS))
        return FAULT_TELEMETRY_TIMEOUT;
    if (!(state->telemetry.status_flags & XGO_STATUS_SENSOR_VALID))
        return FAULT_INVALID_TELEMETRY;
    if (!safety_bridge_ready(state, now))
        return FAULT_BRIDGE;
    if (fabsf(state->telemetry.pitch_deg) > MAX_PITCH_DEG ||
        fabsf(state->telemetry.roll_deg) > MAX_ROLL_DEG)
        return FAULT_TILT;
    if (state->telemetry.battery_pct < CRITICAL_BATTERY_PCT)
        return FAULT_LOW_BATTERY;
    if (!timestamp_fresh(now, state->operator_rx_ns, TELEOP_TIMEOUT_MS))
        return FAULT_OPERATOR_TIMEOUT;
    return FAULT_NONE;
}

static bool safety_recovery_conditions(const RobotStateSnapshot *state, uint64_t now)
{
    return state->child_alive && safety_telemetry_current(state, now) &&
           !(state->telemetry.status_flags & XGO_STATUS_BRIDGE_ERROR) &&
           timestamp_fresh(now, state->operator_rx_ns, TELEOP_TIMEOUT_MS) &&
           fabsf(state->telemetry.pitch_deg) <= RECOVERY_TILT_DEG &&
           fabsf(state->telemetry.roll_deg) <= RECOVERY_TILT_DEG &&
           state->telemetry.battery_pct >= RECOVERY_BATTERY_PCT;
}

void safety_latch_fault(RobotStateSnapshot *state, FaultCode fault, uint64_t now)
{
    if (state->safety == SAFETY_SHUTDOWN)
        return;
    state->reset_requested = false;
    if (state->safety == SAFETY_EMERGENCY)
        return;
    state->safety = SAFETY_EMERGENCY;
    state->fault = fault;
    state->fault_ns = now;
    state->estop_sent_ns = 0;
    state->reset_sent_ns = 0;
    ++state->generation;
}

bool safety_bridge_ready(const RobotStateSnapshot *snapshot, uint64_t now)
{
    return snapshot->child_alive && safety_telemetry_current(snapshot, now) &&
           (snapshot->telemetry.status_flags &
            (XGO_STATUS_READY | XGO_STATUS_EMERGENCY_ACTIVE | XGO_STATUS_BRIDGE_ERROR)) ==
               XGO_STATUS_READY;
}

bool safety_motion_allowed(const RobotStateSnapshot *snapshot, uint64_t now)
{
    return !snapshot->runtime_failed && snapshot->safety == SAFETY_NORMAL &&
           safety_fault(snapshot, now) == FAULT_NONE;
}

bool safety_reset_allowed(const RobotStateSnapshot *state, uint64_t now)
{
    return state->safety == SAFETY_EMERGENCY && state->estop_sent_ns != 0 &&
           state->telemetry.timestamp_ns > state->estop_sent_ns &&
           (state->telemetry.status_flags & XGO_STATUS_EMERGENCY_ACTIVE) &&
           safety_recovery_conditions(state, now);
}

void safety_evaluate(RobotStateSnapshot *state, uint64_t now)
{
    FaultCode fault = safety_fault(state, now);
    switch (state->safety) {
    case SAFETY_STARTING:
        if (state->bridge_initialized) {
            if (fault == FAULT_NONE)
                state->safety = SAFETY_NORMAL;
            else if (fault != FAULT_OPERATOR_TIMEOUT)
                safety_latch_fault(state, fault, now);
        } else if (state->telemetry_rx_ns != 0 &&
                   (state->telemetry.status_flags &
                    (XGO_STATUS_EMERGENCY_ACTIVE | XGO_STATUS_BRIDGE_ERROR))) {
            safety_latch_fault(state, FAULT_BRIDGE, now);
        }
        break;
    case SAFETY_NORMAL:
        if (fault != FAULT_NONE)
            safety_latch_fault(state, fault, now);
        break;
    case SAFETY_EMERGENCY:
        if (state->reset_requested) {
            state->reset_requested = false;
            if (safety_reset_allowed(state, now)) {
                state->safety = SAFETY_RECOVERY;
                state->recovery_started_ns = now;
            }
        }
        break;
    case SAFETY_RECOVERY:
        if (!safety_recovery_conditions(state, now) ||
            !timestamp_fresh(now, state->recovery_started_ns, RECOVERY_TIMEOUT_MS)) {
            safety_latch_fault(state, FAULT_RECOVERY_TIMEOUT, now);
        } else if (state->reset_sent_ns != 0 &&
                   state->telemetry.timestamp_ns > state->reset_sent_ns &&
                   safety_bridge_ready(state, now)) {
            state->safety = SAFETY_NORMAL;
            state->fault = FAULT_NONE;
            ++state->generation;
        }
        break;
    case SAFETY_SHUTDOWN:
        break;
    }
}

const char *safety_state_name(SafetyState value)
{
    static const char *const names[] = {
        "STARTING", "NORMAL", "EMERGENCY", "RECOVERY", "SHUTDOWN"
    };
    return names[value];
}

const char *fault_name(FaultCode value)
{
    static const char *const names[] = {
        "none", "tilt", "operator_timeout", "telemetry_timeout", "invalid_telemetry",
        "bridge", "low_battery", "operator_estop", "recovery_timeout"
    };
    return names[value];
}
