
#ifndef XGO_IPC_PROTOCOL_H
#define XGO_IPC_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * Protocol configuration
 * ========================================================================= */

#define XGO_PROTOCOL_VERSION  1U

#define XGO_CMD_MSG_SIZE      32U
#define XGO_TELEM_MSG_SIZE    32U

/*
 * POSIX message queue priorities.
 * Higher values have higher priority.
 */
#define XGO_MQ_PRIORITY_NORMAL    1U
#define XGO_MQ_PRIORITY_EMERGENCY 2U


/* =========================================================================
 * Command opcodes (C -> Python)
 * ========================================================================= */

typedef enum
{
    XGO_CMD_MOVE = 1,
    XGO_CMD_HEIGHT,
    XGO_CMD_ACTION,
    XGO_CMD_ESTOP,
    XGO_CMD_SHUTDOWN,
    XGO_CMD_HEARTBEAT,
    XGO_CMD_RESET

} XgoCommandType;


/* =========================================================================
 * Telemetry status flags (Python -> C)
 * ========================================================================= */

typedef enum
{
    XGO_STATUS_READY            = (1U << 0),
    XGO_STATUS_SENSOR_VALID     = (1U << 1),
    XGO_STATUS_EMERGENCY_ACTIVE = (1U << 2),
    XGO_STATUS_BRIDGE_ERROR     = (1U << 3)

} XgoStatusFlags;


/* =========================================================================
 * Message structures
 * ========================================================================= */

/*
 * Command message.
 *
 * Direction: C supervisor -> Python bridge
 *
 * The field order is intentional. It produces a naturally aligned
 * 32-byte structure on the target ARM64 platform.
 */
typedef struct
{
    uint32_t sequence;
    uint16_t version;
    uint16_t command_type;

    float vx;
    float wz;
    float height_z;

    uint32_t action_id;

    uint64_t timestamp_ns;

} XgoCmdMsg;


/*
 * Telemetry message.
 *
 * Direction: Python bridge -> C supervisor
 */
typedef struct
{
    uint32_t sequence;
    uint16_t version;
    uint16_t status_flags;

    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    float battery_pct;

    uint64_t timestamp_ns;

} XgoTelemMsg;


/* =========================================================================
 * Message initialization
 * ========================================================================= */

/*
 * Initialize a command message.
 * All fields are zero-initialized.
 *
 * The caller must subsequently populate the fields required
 * by the selected command.
 */
void xgo_cmd_init(
    XgoCmdMsg *msg,
    XgoCommandType command_type,
    uint32_t sequence
);


/*
 * Initialize a telemetry message.
 */
void xgo_telem_init(
    XgoTelemMsg *msg,
    uint32_t sequence
);


/* =========================================================================
 * Validation
 * ========================================================================= */

/*
 * Validate the protocol structure and its semantic fields.
 *
 * These functions do not check application-specific safety limits
 * or message freshness.
 */
bool xgo_cmd_is_valid(const XgoCmdMsg *msg);

bool xgo_telem_is_valid(const XgoTelemMsg *msg);


/* =========================================================================
 * Timestamp utilities
 * ========================================================================= */

/*
 * Obtain the current CLOCK_MONOTONIC timestamp in nanoseconds.
 *
 * Returns false if clock_gettime() fails.
 */
bool xgo_monotonic_now_ns(uint64_t *timestamp_ns);


/* =========================================================================
 * Message queue utilities
 * ========================================================================= */

/*
 * Return the POSIX MQ priority corresponding to a command.
 */
unsigned int xgo_cmd_mq_priority(const XgoCmdMsg *msg);

#endif /* XGO_IPC_PROTOCOL_H */
