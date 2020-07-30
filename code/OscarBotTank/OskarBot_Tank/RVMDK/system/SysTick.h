#include <stddef.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "system/SysIrq.h"

#ifndef __SYS_TICK_H
#define __SYS_TICK_H

#ifdef __cplusplus
 extern "C" {
#endif

#define SYS_TICK_FREQ   (1000u)

extern volatile uint32_t sysTickMs;

extern uint32_t sysTickValMax;
extern uint32_t sysTickValPerMs;
extern uint32_t sysTickValPerUs;

void sysTickInit(void);

static inline void sysTickInc(void) {
    sysTickMs++;
}

static inline uint32_t sysTickCurrentMs(void) {
    return sysTickMs;
}

static inline uint32_t sysTickGetMs(uint32_t *tickMs) {
    uint32_t currentSysTickMs = sysTickMs;
    if (tickMs != NULL) {
        *tickMs = currentSysTickMs;
    }
    return currentSysTickMs;
}

static inline uint32_t sysTickSetMs(const uint32_t tickMs) {
    irqLock();

    uint32_t currentSysTickMs = sysTickMs;
    sysTickMs = tickMs;

    irqUnLock();
    return currentSysTickMs;
}

#ifdef __cplusplus
}
#endif

#endif /* __SYS_TICK_H */
