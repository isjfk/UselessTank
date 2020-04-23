#include <stdint.h>
#include <stdbool.h>

#include "stm32f10x.h"

#include "SysTick.h"

#ifndef __SYS_TIME_H
#define __SYS_TIME_H

#ifdef __cplusplus
 extern "C" {
#endif

static inline uint32_t sysTimeCurrentMs(void) {
    return sysTickCurrentMs();
}

static inline void sysTimeGetMs(uint32_t *timeMs) {
    sysTickGetMs(timeMs);
}

typedef struct {
    uint32_t intervalTimeMs;
    uint32_t prevTimeMs;
} SysTimeInterval;

int sysTimeIntervalInit(SysTimeInterval *interval, uint32_t intervalTimeMs);
bool sysTimeIsOnInterval(SysTimeInterval *interval);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_TIME_H */
