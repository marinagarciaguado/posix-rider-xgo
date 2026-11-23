#include "shared_state.h"

#include "safety.h"
#include "scheduler_utils.h"

static pthread_mutex_t lock;
static RobotStateSnapshot state;

int shared_state_init(void)
{
    state = (RobotStateSnapshot){ .safety = SAFETY_STARTING, .child_alive = true };
    return init_shared_mutex(&lock);
}

RobotStateSnapshot shared_state_snapshot(void)
{
    pthread_mutex_lock(&lock);
    RobotStateSnapshot copy = state;
    pthread_mutex_unlock(&lock);
    return copy;
}

void shared_state_telemetry(const XgoTelemMsg *msg, uint64_t now)
{
    pthread_mutex_lock(&lock);
    state.telemetry = *msg;
    state.telemetry_rx_ns = now;
    if (safety_bridge_ready(&state, now))
        state.bridge_initialized = true;
    pthread_mutex_unlock(&lock);
}

void shared_state_operator_seen(uint64_t now)
{
    pthread_mutex_lock(&lock);
    state.operator_rx_ns = now;
    pthread_mutex_unlock(&lock);
}

void shared_state_child_exited(void)
{
    pthread_mutex_lock(&lock);
    state.child_alive = false;
    safety_latch_fault(&state, FAULT_BRIDGE, monotonic_ns());
    pthread_mutex_unlock(&lock);
}

void shared_state_fail(void)
{
    pthread_mutex_lock(&lock);
    state.runtime_failed = true;
    pthread_mutex_unlock(&lock);
}

void shared_state_shutdown(void)
{
    pthread_mutex_lock(&lock);
    state.safety = SAFETY_SHUTDOWN;
    ++state.generation;
    pthread_mutex_unlock(&lock);
}

void shared_state_estop(FaultCode fault, uint64_t now)
{
    pthread_mutex_lock(&lock);
    safety_latch_fault(&state, fault, now);
    pthread_mutex_unlock(&lock);
}

void shared_state_request_reset(void)
{
    pthread_mutex_lock(&lock);
    if (safety_reset_allowed(&state, monotonic_ns()))
        state.reset_requested = true;
    pthread_mutex_unlock(&lock);
}

void shared_state_note_estop(uint64_t generation, uint64_t now)
{
    pthread_mutex_lock(&lock);
    if (state.safety == SAFETY_EMERGENCY && state.generation == generation)
        state.estop_sent_ns = now;
    pthread_mutex_unlock(&lock);
}

void shared_state_note_reset(uint64_t generation, uint64_t now)
{
    pthread_mutex_lock(&lock);
    if (state.safety == SAFETY_RECOVERY && state.generation == generation)
        state.reset_sent_ns = now;
    pthread_mutex_unlock(&lock);
}

void shared_state_evaluate(void)
{
    pthread_mutex_lock(&lock);
    safety_evaluate(&state, monotonic_ns());
    pthread_mutex_unlock(&lock);
}

void shared_state_destroy(void)
{
    pthread_mutex_destroy(&lock);
}
