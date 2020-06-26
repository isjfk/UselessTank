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
} SysTimeLoop;

int sysTimeLoopStart(SysTimeLoop *loop, uint32_t intervalTimeMs);
static inline int sysTimeLoopStop(SysTimeLoop *loop) {
    if (!loop) {
        return -1;
    }
    loop->prevTimeMs = 0;
    return 0;
}
static inline bool sysTimeLoopIsStart(SysTimeLoop *loop) {
    return loop && (loop->prevTimeMs > 0) && (loop->intervalTimeMs > 0);
}
bool sysTimeLoopShouldEnter(SysTimeLoop *loop);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_TIME_H */
