#include "SysTick.h"

volatile uint32_t sysTickMs;

uint32_t sysTickValMax;
uint32_t sysTickValPerMs;
uint32_t sysTickValPerUs;

void sysTickInit(void) {
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / SYS_TICK_FREQ)) {  // Setup SysTick Timer for 1 msec interrupts
        while (1);                                          // Handle Error
	}

    sysTickValMax = SystemCoreClock / SYS_TICK_FREQ;
    sysTickValPerMs = SystemCoreClock / 1000;
    sysTickValPerUs = SystemCoreClock / 1000000;
}
