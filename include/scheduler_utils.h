#ifndef XGO_SCHEDULER_UTILS_H
#define XGO_SCHEDULER_UTILS_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#define NS_PER_MS UINT64_C(1000000)

uint64_t monotonic_ns(void);
bool timestamp_fresh(uint64_t now, uint64_t stamp, unsigned max_ms);
bool sequence_newer(uint32_t value, uint32_t previous);
int init_shared_mutex(pthread_mutex_t *mutex);
void configure_safety_scheduler(bool use_fifo);

#endif
