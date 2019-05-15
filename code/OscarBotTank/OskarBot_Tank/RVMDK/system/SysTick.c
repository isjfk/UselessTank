#include "SysTick.h"

#define TICK_FREQ (1000u)

volatile uint32_t sysTickMs;

void sysTickInit(void) {
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / TICK_FREQ)) {      // Setup SysTick Timer for 1 msec interrupts
        while (1);                                          // Handle Error
	}
}

inline void sysTickInc(void) {
    sysTickMs++;
}

inline uint32_t sysTickCurrent(void) {
    return sysTickMs;
}

inline void sysTickGetMs(uint32_t *tick) {
    *tick = sysTickMs;
}
