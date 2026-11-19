#ifndef RIDER_PROTOCOL_H
#define RIDER_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Formato de 96 bytes, enteros big-endian, sin padding de structs.
   Compartido por los dos programas C; todavia no es una API del SDK. */
enum { RIDER_WIRE_SIZE = 96, RIDER_VERSION = 1, RIDER_MQ_MAX = 10,
       RIDER_NORMAL_MAX = 6, RIDER_RESP_NORMAL_MAX = 8,
       RIDER_ARMADO = 1, RIDER_MUESTRA_VALIDA = 2,
       RIDER_LEASE_MS = 500, RIDER_TIMEOUT_MS = 600,
       RIDER_MAX_MS = 3000, RIDER_MUESTREO_MS = 200, RIDER_EDAD_MAX_MS = 600 };
enum {
    C_LATIDO = 1, C_PARAR, C_REARMAR, C_MOVER, C_SALIR,
    R_LISTO = 16, R_ESTADO, R_INICIO, R_FIN, R_PARADA, R_REARME,
    R_DESCARTE, R_FALLO, R_SALIDA
};
enum { MOTIVO_NINGUNO, MOTIVO_CADUCADO, MOTIVO_GENERACION,
       MOTIVO_NO_ARMADO, MOTIVO_OCUPADO, MOTIVO_SENSOR,
       MOTIVO_WATCHDOG, MOTIVO_SATURACION, MOTIVO_PROTOCOLO };
typedef struct {
    uint32_t tipo, flags;
    uint64_t secuencia, generacion, orden_id;
    int64_t caduca_ms;
    uint32_t duracion_ms;
    int direccion;
    uint32_t sensor, motivo;
    uint64_t muestra;
    int64_t muestra_ms, instante_ms, fin_ms;
} Mensaje;

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
static inline void rider_codificar(const Mensaje *m, unsigned char *p) {
    rider_put32(p, UINT32_C(0x52445231)); /* RDR1 */
    p[4] = 0; p[5] = RIDER_VERSION; p[6] = 0; p[7] = RIDER_WIRE_SIZE;
    rider_put32(p + 8, m->tipo); rider_put32(p + 12, m->flags);
    rider_put64(p + 16, m->secuencia); rider_put64(p + 24, m->generacion);
    rider_put64(p + 32, m->orden_id); rider_put64(p + 40, (uint64_t)m->caduca_ms);
    rider_put32(p + 48, m->duracion_ms); rider_put32(p + 52, (uint32_t)(m->direccion + 1));
    rider_put32(p + 56, m->sensor); rider_put32(p + 60, m->motivo);
    rider_put64(p + 64, m->muestra); rider_put64(p + 72, (uint64_t)m->muestra_ms);
    rider_put64(p + 80, (uint64_t)m->instante_ms); rider_put64(p + 88, (uint64_t)m->fin_ms);
}
static inline bool rider_decodificar(const unsigned char *p, size_t n, Mensaje *m) {
    if (n != RIDER_WIRE_SIZE || rider_get32(p) != UINT32_C(0x52445231) ||
        p[4] != 0 || p[5] != RIDER_VERSION || p[6] != 0 || p[7] != RIDER_WIRE_SIZE ||
        rider_get32(p + 52) > 2 || rider_get64(p + 40) > INT64_MAX ||
        rider_get64(p + 72) > INT64_MAX || rider_get64(p + 80) > INT64_MAX ||
        rider_get64(p + 88) > INT64_MAX) return false;
    *m = (Mensaje){
        .tipo = rider_get32(p + 8), .flags = rider_get32(p + 12),
        .secuencia = rider_get64(p + 16), .generacion = rider_get64(p + 24),
        .orden_id = rider_get64(p + 32), .caduca_ms = (int64_t)rider_get64(p + 40),
        .duracion_ms = rider_get32(p + 48), .direccion = (int)rider_get32(p + 52) - 1,
        .sensor = rider_get32(p + 56), .motivo = rider_get32(p + 60),
        .muestra = rider_get64(p + 64), .muestra_ms = (int64_t)rider_get64(p + 72),
        .instante_ms = (int64_t)rider_get64(p + 80), .fin_ms = (int64_t)rider_get64(p + 88)
    };
    return m->secuencia != 0 && m->generacion != 0 && m->flags <= 3 &&
           m->sensor <= 2 && m->motivo <= MOTIVO_PROTOCOLO &&
           ((m->tipo >= C_LATIDO && m->tipo <= C_SALIR) ||
            (m->tipo >= R_LISTO && m->tipo <= R_SALIDA));
}

#endif
