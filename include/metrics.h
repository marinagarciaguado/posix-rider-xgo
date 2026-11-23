#ifndef XGO_METRICS_H
#define XGO_METRICS_H

#include "shared_state.h"

typedef struct {
    RobotStateSnapshot state;
    uint64_t timestamp_ns;
    uint64_t cycle;
    uint64_t period_ns;
    int overruns;
} SafetyMetric;

int metrics_start(const char *path);
void metrics_publish(const SafetyMetric *sample);
int metrics_stop(void);

#endif
