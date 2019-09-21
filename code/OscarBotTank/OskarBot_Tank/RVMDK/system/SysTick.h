#include <stdint.h>

#include "stm32f10x.h"

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

static inline void sysTickGetMs(uint32_t *tick) {
    *tick = sysTickMs;
}

#ifdef __cplusplus
}
#endif

#endif /* __SYS_TICK_H */
