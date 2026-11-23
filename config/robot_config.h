#ifndef XGO_ROBOT_CONFIG_H
#define XGO_ROBOT_CONFIG_H

#define SAFETY_PERIOD_NS        20000000L
#define TELEOP_TIMEOUT_MS       200
#define TELEMETRY_TIMEOUT_MS    200
#define MAX_FORWARD_SPEED       0.30f
#define MAX_TURN_SPEED          0.50f
#define MAX_PITCH_DEG           18.0f
#define MAX_ROLL_DEG            18.0f
#define CMD_QUEUE_CAPACITY      10

#define CMD_MQ_NAME             "/xgo_cmd_mq"
#define TELEM_MQ_NAME           "/xgo_telem_mq"
#define MQ_CAPACITY             10
#define DEFAULT_BIND_ADDRESS    "127.0.0.1"
#define DEFAULT_OPERATOR_IP     "127.0.0.1"
#define DEFAULT_TELEOP_PORT     45800
#define DEFAULT_OPERATOR_PORT   45801
#define IPC_PERIOD_MS           5
#define HEARTBEAT_PERIOD_MS     50
#define COMMAND_MAX_AGE_MS      150
#define STARTUP_TIMEOUT_MS      5000
#define BRIDGE_STARTUP_GRACE_MS 3000
#define BRIDGE_TELEMETRY_MS     50
#define RECOVERY_TIMEOUT_MS     1000
#define CHILD_EXIT_TIMEOUT_MS   750
#define BRIDGE_STOP_TIMEOUT_MS  250
#define CRITICAL_BATTERY_PCT    15.0f
#define RECOVERY_BATTERY_PCT    20.0f
#define RECOVERY_TILT_DEG       12.0f
#define MIN_HEIGHT_MM          60.0f
#define MAX_HEIGHT_MM          120.0f
#define DEADLINE_TOLERANCE_NS   2000000LL
#define METRICS_CAPACITY        256

#endif
