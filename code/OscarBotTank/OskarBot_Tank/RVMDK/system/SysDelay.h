#include <stdint.h>

#include "stm32f10x.h"

#ifndef __SYS_DELAY_H
#define __SYS_DELAY_H

#ifdef __cplusplus
 extern "C" {
#endif

void sysDelayUs(uint32_t timeUs);
void sysDelayMs(uint32_t timeMs);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_DELAY_H */
