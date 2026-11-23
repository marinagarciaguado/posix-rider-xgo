#include "ipc_bridge.h"

#include "cmd_queue.h"
#include "robot_config.h"
#include "safety.h"
#include "scheduler_utils.h"
#include "shared_state.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>

typedef struct {
    bool telemetry_seen;
    uint32_t telemetry_sequence;
    uint32_t next_sequence;
    uint64_t last_heartbeat_ns;
    uint64_t generation;
} Dispatcher;

int ipc_bridge_open(IpcBridge *bridge)
{
    struct mq_attr attributes = { .mq_maxmsg = MQ_CAPACITY, .mq_msgsize = XGO_CMD_MSG_SIZE };
    bridge->commands = mq_open(CMD_MQ_NAME, O_WRONLY | O_CREAT | O_EXCL | O_NONBLOCK,
                               0600, &attributes);
    if (bridge->commands == (mqd_t)-1)
        return -1;

    attributes.mq_msgsize = XGO_TELEM_MSG_SIZE;
    bridge->telemetry = mq_open(TELEM_MQ_NAME, O_RDONLY | O_CREAT | O_EXCL | O_NONBLOCK,
                                0600, &attributes);
    if (bridge->telemetry == (mqd_t)-1) {
        int error = errno;
        mq_close(bridge->commands);
        mq_unlink(CMD_MQ_NAME);
        errno = error;
        return -1;
    }
    return 0;
}

void ipc_bridge_close(IpcBridge *bridge)
{
    mq_close(bridge->commands);
    mq_close(bridge->telemetry);
    mq_unlink(CMD_MQ_NAME);
    mq_unlink(TELEM_MQ_NAME);
}

static bool receive_telemetry(IpcBridge *bridge, Dispatcher *dispatcher)
{
    for (unsigned i = 0; i < MQ_CAPACITY; ++i) {
        XgoTelemMsg message;
        ssize_t bytes = mq_receive(bridge->telemetry, (char *)&message, sizeof(message), NULL);
        if (bytes == -1)
            return errno == EAGAIN || errno == EINTR;

        uint64_t now = monotonic_ns();
        if (bytes != sizeof(message) || !xgo_telem_is_valid(&message) ||
            ((message.status_flags & XGO_STATUS_SENSOR_VALID) &&
             (message.pitch_deg < -180.0f || message.pitch_deg > 180.0f ||
              message.roll_deg < -180.0f || message.roll_deg > 180.0f))) {
            shared_state_estop(FAULT_INVALID_TELEMETRY, now);
            continue;
        }
        if (!timestamp_fresh(now, message.timestamp_ns, TELEMETRY_TIMEOUT_MS) ||
            (dispatcher->telemetry_seen &&
             !sequence_newer(message.sequence, dispatcher->telemetry_sequence)))
            continue;

        dispatcher->telemetry_seen = true;
        dispatcher->telemetry_sequence = message.sequence;
        shared_state_telemetry(&message, now);
    }
    return true;
}

/* Return 1 on send, 0 on transient backpressure, -1 on transport failure. */
static int send_command(IpcBridge *bridge, Dispatcher *dispatcher, XgoCmdMsg *message)
{
    message->sequence = dispatcher->next_sequence;
    if (mq_send(bridge->commands, (const char *)message, sizeof(*message),
                xgo_cmd_mq_priority(message)) == 0) {
        ++dispatcher->next_sequence;
        return 1;
    }
    return errno == EAGAIN || errno == EINTR ? 0 : -1;
}

static int send_control(IpcBridge *bridge, Dispatcher *dispatcher,
                        XgoCommandType command, uint64_t now)
{
    XgoCmdMsg message;
    xgo_cmd_init(&message, command, 0);
    message.timestamp_ns = now;
    return send_command(bridge, dispatcher, &message);
}

static bool dispatch_cycle(IpcBridge *bridge, Dispatcher *dispatcher,
                           const RobotStateSnapshot *snapshot)
{
    uint64_t now = monotonic_ns();
    if (snapshot->generation != dispatcher->generation) {
        dispatcher->generation = snapshot->generation;
        cmd_queue_discard_before(dispatcher->generation);
    }
    if (snapshot->safety == SAFETY_EMERGENCY && snapshot->estop_sent_ns == 0) {
        int result = send_control(bridge, dispatcher, XGO_CMD_ESTOP, now);
        if (result > 0)
            shared_state_note_estop(snapshot->generation, now);
        return result >= 0;
    }

    struct mq_attr attributes;
    if (mq_getattr(bridge->commands, &attributes) == -1)
        return false;
    /* Keep one slot available for ESTOP and bound the normal backlog. */
    if (attributes.mq_curmsgs >= attributes.mq_maxmsg - 1)
        return true;

    if (now - dispatcher->last_heartbeat_ns >= HEARTBEAT_PERIOD_MS * NS_PER_MS) {
        int result = send_control(bridge, dispatcher, XGO_CMD_HEARTBEAT, now);
        if (result > 0)
            dispatcher->last_heartbeat_ns = now;
        return result >= 0;
    }
    if (snapshot->safety == SAFETY_RECOVERY && snapshot->reset_sent_ns == 0) {
        int result = send_control(bridge, dispatcher, XGO_CMD_RESET, now);
        if (result > 0)
            shared_state_note_reset(snapshot->generation, now);
        return result >= 0;
    }
    if (attributes.mq_curmsgs >= 2 || !safety_motion_allowed(snapshot, now))
        return true;

    QueuedCommand queued;
    if (!cmd_queue_pop(&queued))
        return true;

    RobotStateSnapshot latest = shared_state_snapshot();
    now = monotonic_ns();
    if (queued.generation != latest.generation ||
        !safety_motion_allowed(&latest, now) ||
        !timestamp_fresh(now, queued.message.timestamp_ns, COMMAND_MAX_AGE_MS))
        return true;
    return send_command(bridge, dispatcher, &queued.message) >= 0;
}

static void wait_for_next_cycle(void)
{
    const struct timespec delay = { .tv_nsec = IPC_PERIOD_MS * 1000000L };
    nanosleep(&delay, NULL);
}

static void send_shutdown(IpcBridge *bridge, Dispatcher *dispatcher)
{
    uint64_t deadline = monotonic_ns() + BRIDGE_STOP_TIMEOUT_MS * NS_PER_MS;
    bool stopped = false;
    do {
        XgoCommandType opcode = stopped ? XGO_CMD_SHUTDOWN : XGO_CMD_ESTOP;
        int result = send_control(bridge, dispatcher, opcode, monotonic_ns());
        if (result < 0)
            break;
        if (result > 0) {
            if (stopped)
                return;
            stopped = true;
            continue;
        }
        wait_for_next_cycle();
    } while (monotonic_ns() < deadline);
    fputs("Could not queue bridge shutdown; process manager will enforce a timeout\n", stderr);
}

void *ipc_worker(void *argument)
{
    IpcBridge *bridge = argument;
    Dispatcher dispatcher = { 0 };
    for (;;) {
        RobotStateSnapshot snapshot = shared_state_snapshot();
        if (snapshot.safety == SAFETY_SHUTDOWN)
            break;
        if (!receive_telemetry(bridge, &dispatcher)) {
            perror("Receiving telemetry");
            shared_state_fail();
            break;
        }
        snapshot = shared_state_snapshot();
        if (!dispatch_cycle(bridge, &dispatcher, &snapshot)) {
            perror("Dispatching a command");
            shared_state_fail();
            break;
        }
        wait_for_next_cycle();
    }
    if (shared_state_snapshot().child_alive)
        send_shutdown(bridge, &dispatcher);
    return NULL;
}
