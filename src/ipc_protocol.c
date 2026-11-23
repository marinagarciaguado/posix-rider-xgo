
#include "ipc_protocol.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "The native XGO IPC record layout requires a little-endian target"
#endif


/* =========================================================================
 * Compile-time protocol verification
 * ========================================================================= */

/*
 * These assertions guarantee that the C structures match
 * the expected binary communication layout.
 */

_Static_assert(sizeof(float) == 4,
               "Protocol requires 32-bit floats");

_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128,
               "Protocol requires IEEE-754 binary32 floats");

_Static_assert(sizeof(XgoCmdMsg) == XGO_CMD_MSG_SIZE,
               "XgoCmdMsg must be exactly 32 bytes");

_Static_assert(sizeof(XgoTelemMsg) == XGO_TELEM_MSG_SIZE,
               "XgoTelemMsg must be exactly 32 bytes");


/* Verify command message field offsets. */

_Static_assert(offsetof(XgoCmdMsg, sequence) == 0,
               "Invalid XgoCmdMsg sequence offset");

_Static_assert(offsetof(XgoCmdMsg, version) == 4,
               "Invalid XgoCmdMsg version offset");

_Static_assert(offsetof(XgoCmdMsg, command_type) == 6,
               "Invalid XgoCmdMsg command_type offset");

_Static_assert(offsetof(XgoCmdMsg, vx) == 8,
               "Invalid XgoCmdMsg vx offset");

_Static_assert(offsetof(XgoCmdMsg, wz) == 12,
               "Invalid XgoCmdMsg wz offset");

_Static_assert(offsetof(XgoCmdMsg, height_z) == 16,
               "Invalid XgoCmdMsg height_z offset");

_Static_assert(offsetof(XgoCmdMsg, action_id) == 20,
               "Invalid XgoCmdMsg action_id offset");

_Static_assert(offsetof(XgoCmdMsg, timestamp_ns) == 24,
               "Invalid XgoCmdMsg timestamp offset");


/* Verify telemetry message field offsets. */

_Static_assert(offsetof(XgoTelemMsg, sequence) == 0,
               "Invalid XgoTelemMsg sequence offset");

_Static_assert(offsetof(XgoTelemMsg, version) == 4,
               "Invalid XgoTelemMsg version offset");

_Static_assert(offsetof(XgoTelemMsg, status_flags) == 6,
               "Invalid XgoTelemMsg status_flags offset");

_Static_assert(offsetof(XgoTelemMsg, pitch_deg) == 8,
               "Invalid XgoTelemMsg pitch offset");

_Static_assert(offsetof(XgoTelemMsg, roll_deg) == 12,
               "Invalid XgoTelemMsg roll offset");

_Static_assert(offsetof(XgoTelemMsg, yaw_deg) == 16,
               "Invalid XgoTelemMsg yaw offset");

_Static_assert(offsetof(XgoTelemMsg, battery_pct) == 20,
               "Invalid XgoTelemMsg battery offset");

_Static_assert(offsetof(XgoTelemMsg, timestamp_ns) == 24,
               "Invalid XgoTelemMsg timestamp offset");


/* =========================================================================
 * Internal validation helpers
 * ========================================================================= */

/*
 * Check whether a command opcode is recognized.
 */
static bool is_known_command(uint16_t command_type)
{
    switch (command_type)
    {
        case XGO_CMD_MOVE:
        case XGO_CMD_HEIGHT:
        case XGO_CMD_ACTION:
        case XGO_CMD_ESTOP:
        case XGO_CMD_SHUTDOWN:
        case XGO_CMD_HEARTBEAT:
        case XGO_CMD_RESET:
            return true;

        default:
            return false;
    }
}


/* =========================================================================
 * Message initialization
 * ========================================================================= */

void xgo_cmd_init(XgoCmdMsg *msg, XgoCommandType command_type, uint32_t sequence)
{
    if (msg == NULL)
        return;

    memset(msg, 0, sizeof(*msg));

    msg->sequence = sequence;
    msg->version = XGO_PROTOCOL_VERSION;
    msg->command_type = (uint16_t)command_type;
}


void xgo_telem_init(XgoTelemMsg *msg, uint32_t sequence)
{
    if (msg == NULL)
        return;

    memset(msg, 0, sizeof(*msg));

    msg->sequence = sequence;
    msg->version = XGO_PROTOCOL_VERSION;
}


/* =========================================================================
 * Command validation
 * ========================================================================= */

bool xgo_cmd_is_valid(const XgoCmdMsg *msg)
{
    if (msg == NULL)
        return false;

    if (msg->version != XGO_PROTOCOL_VERSION)
        return false;

    if (!is_known_command(msg->command_type))
        return false;

    if (msg->timestamp_ns == 0)
        return false;

    /*
     * All floating-point fields must contain finite values,
     * including fields unused by the selected opcode.
     */
    if (!isfinite(msg->vx) ||
        !isfinite(msg->wz) ||
        !isfinite(msg->height_z))
    {
        return false;
    }

    /*
     * Command-specific validation.
     */
    switch (msg->command_type)
    {
        case XGO_CMD_MOVE:
            return true;

        case XGO_CMD_HEIGHT:
            return true;

        case XGO_CMD_ACTION:
            return msg->action_id != 0;

        case XGO_CMD_ESTOP:
        case XGO_CMD_SHUTDOWN:
        case XGO_CMD_HEARTBEAT:
        case XGO_CMD_RESET:
            return true;

        default:
            return false;
    }
}


/* =========================================================================
 * Telemetry validation
 * ========================================================================= */

bool xgo_telem_is_valid(const XgoTelemMsg *msg)
{
    if (msg == NULL)
        return false;

    if (msg->version != XGO_PROTOCOL_VERSION)
        return false;

    if (msg->timestamp_ns == 0)
        return false;

    const uint16_t allowed_flags =
        XGO_STATUS_READY |
        XGO_STATUS_SENSOR_VALID |
        XGO_STATUS_EMERGENCY_ACTIVE |
        XGO_STATUS_BRIDGE_ERROR;

    /*
     * Reject undefined status bits.
     */
    if ((msg->status_flags & ~allowed_flags) != 0)
        return false;

    /*
     * Reject invalid floating-point values.
     */
    if (!isfinite(msg->pitch_deg) ||
        !isfinite(msg->roll_deg) ||
        !isfinite(msg->yaw_deg) ||
        !isfinite(msg->battery_pct))
    {
        return false;
    }

    /*
     * Battery percentage must be within its physical range.
     */
    if (msg->battery_pct < 0.0f ||
        msg->battery_pct > 100.0f)
    {
        return false;
    }

    return true;
}


/* =========================================================================
 * Timestamp utilities
 * ========================================================================= */

bool xgo_monotonic_now_ns(uint64_t *timestamp_ns)
{
    if (timestamp_ns == NULL)
        return false;

    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return false;

    *timestamp_ns =
        ((uint64_t)now.tv_sec * 1000000000ULL) +
        (uint64_t)now.tv_nsec;

    return true;
}


/* =========================================================================
 * Message queue utilities
 * ========================================================================= */

unsigned int xgo_cmd_mq_priority(const XgoCmdMsg *msg)
{
    if (msg == NULL)
        return XGO_MQ_PRIORITY_NORMAL;

    if (msg->command_type == XGO_CMD_ESTOP)
        return XGO_MQ_PRIORITY_EMERGENCY;

    return XGO_MQ_PRIORITY_NORMAL;
}
