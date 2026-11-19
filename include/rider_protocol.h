#ifndef RIDER_PROTOCOL_H
#define RIDER_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Fixed 96-byte format, big-endian integers, no structure padding.
   Shared by both C programs; this is not a robot SDK interface. */
enum { RIDER_WIRE_SIZE = 96, RIDER_VERSION = 1, RIDER_MQ_MAX = 10,
       RIDER_NORMAL_MAX = 6, RIDER_RESP_NORMAL_MAX = 8,
       RIDER_ARMED = 1, RIDER_SAMPLE_VALID = 2,
       RIDER_LEASE_MS = 500, RIDER_TIMEOUT_MS = 600,
       RIDER_MAX_MS = 3000, RIDER_SAMPLE_PERIOD_MS = 200, RIDER_MAX_SAMPLE_AGE_MS = 600 };
enum {
    C_HEARTBEAT = 1, C_STOP, C_ARM, C_MOVE, C_QUIT,
    R_READY = 16, R_STATUS, R_STARTED, R_COMPLETED, R_STOPPED, R_ARMED,
    R_REJECTED, R_FAULT, R_EXITED
};
enum { REASON_NONE, REASON_EXPIRED, REASON_GENERATION,
       REASON_NOT_ARMED, REASON_BUSY, REASON_SENSOR,
       REASON_WATCHDOG, REASON_SATURATION, REASON_PROTOCOL };
typedef struct {
    uint32_t type, flags;
    uint64_t sequence, generation, command_id;
    int64_t expires_ms;
    uint32_t duration_ms;
    int direction;
    uint32_t sensor, reason;
    uint64_t sample;
    int64_t sample_ms, timestamp_ms, end_ms;
} Message;

static inline void rider_put64(unsigned char *p, uint64_t n) {
    for (int i = 7; i >= 0; --i) { p[i] = (unsigned char)(n & 255); n >>= 8; }
}
static inline uint64_t rider_get64(const unsigned char *p) {
    uint64_t n = 0;
    for (int i = 0; i < 8; ++i) n = (n << 8) | p[i];
    return n;
}
static inline void rider_put32(unsigned char *p, uint32_t n) {
    for (int i = 3; i >= 0; --i) { p[i] = (unsigned char)(n & 255); n >>= 8; }
}
static inline uint32_t rider_get32(const unsigned char *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}
static inline void rider_encode(const Message *m, unsigned char *p) {
    rider_put32(p, UINT32_C(0x52445231)); /* RDR1 */
    p[4] = 0; p[5] = RIDER_VERSION; p[6] = 0; p[7] = RIDER_WIRE_SIZE;
    rider_put32(p + 8, m->type); rider_put32(p + 12, m->flags);
    rider_put64(p + 16, m->sequence); rider_put64(p + 24, m->generation);
    rider_put64(p + 32, m->command_id); rider_put64(p + 40, (uint64_t)m->expires_ms);
    rider_put32(p + 48, m->duration_ms); rider_put32(p + 52, (uint32_t)(m->direction + 1));
    rider_put32(p + 56, m->sensor); rider_put32(p + 60, m->reason);
    rider_put64(p + 64, m->sample); rider_put64(p + 72, (uint64_t)m->sample_ms);
    rider_put64(p + 80, (uint64_t)m->timestamp_ms); rider_put64(p + 88, (uint64_t)m->end_ms);
}
static inline bool rider_is_command(uint32_t type) {
    return type >= C_HEARTBEAT && type <= C_QUIT;
}
static inline bool rider_is_response(uint32_t type) {
    return type >= R_READY && type <= R_EXITED;
}
static inline bool rider_valid_header(const unsigned char *data, size_t size) {
    if (size != RIDER_WIRE_SIZE) return false;
    if (rider_get32(data) != UINT32_C(0x52445231)) return false;
    if (data[4] != 0 || data[5] != RIDER_VERSION) return false;
    return data[6] == 0 && data[7] == RIDER_WIRE_SIZE;
}
static inline bool rider_valid_timestamps(const unsigned char *data) {
    const size_t offsets[] = {40, 72, 80, 88};
    for (size_t i = 0; i < sizeof offsets / sizeof offsets[0]; ++i) {
        if (rider_get64(data + offsets[i]) > INT64_MAX) return false;
    }
    return true;
}
static inline bool rider_valid_fields(const Message *message) {
    if (message->sequence == 0 || message->generation == 0) return false;
    if (message->flags > 3 || message->sensor > 2) return false;
    if (message->reason > REASON_PROTOCOL) return false;
    return rider_is_command(message->type) || rider_is_response(message->type);
}
static inline bool rider_decode(const unsigned char *p, size_t n, Message *m) {
    if (!rider_valid_header(p, n)) return false;
    if (!rider_valid_timestamps(p)) return false;
    if (rider_get32(p + 52) > 2) return false;
    *m = (Message){
        .type = rider_get32(p + 8), .flags = rider_get32(p + 12),
        .sequence = rider_get64(p + 16), .generation = rider_get64(p + 24),
        .command_id = rider_get64(p + 32), .expires_ms = (int64_t)rider_get64(p + 40),
        .duration_ms = rider_get32(p + 48), .direction = (int)rider_get32(p + 52) - 1,
        .sensor = rider_get32(p + 56), .reason = rider_get32(p + 60),
        .sample = rider_get64(p + 64), .sample_ms = (int64_t)rider_get64(p + 72),
        .timestamp_ms = (int64_t)rider_get64(p + 80), .end_ms = (int64_t)rider_get64(p + 88)
    };
    return rider_valid_fields(m);
}

#endif
