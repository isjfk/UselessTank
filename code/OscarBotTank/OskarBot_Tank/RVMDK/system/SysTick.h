#include <stdint.h>

#include "stm32f10x.h"

#ifndef __SYS_TICK_H
#define __SYS_TICK_H

#ifdef __cplusplus
 extern "C" {
#endif

extern volatile uint32_t sysTickMs;

void sysTickInit(void);

inline void sysTickInc(void);
extern void sysTickInc(void);

inline uint32_t sysTickCurrent(void);
extern uint32_t sysTickCurrent(void);

inline void sysTickGetMs(uint32_t *tick);
extern void sysTickGetMs(uint32_t *tick);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_TICK_H */
