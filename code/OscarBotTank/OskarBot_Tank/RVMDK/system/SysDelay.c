#include "SysDelay.h"
#include "SysTick.h"

void sysDelayUs(uint32_t timeUs) {
    uint32_t tickValPrev = SysTick->VAL;
    uint32_t tickValCurr;
    uint32_t tick = 0;

    timeUs = timeUs * sysTickValPerUs;
    while (tick < timeUs) {
        tickValCurr = SysTick->VAL;

        // SysTick is counting down, so tickValPrev is great then tickValCurr.
        tick += (tickValPrev - tickValCurr) + ((tickValPrev < tickValCurr) ? sysTickValMax : 0);
        tickValPrev = tickValCurr;
    }
}

void sysDelayMs(uint32_t timeMs) {
    if (timeMs < 10) {
        sysDelayUs(timeMs * 1000);
    } else {
        uint32_t tickMsStart = sysTickCurrentMs();
        while ((sysTickCurrentMs() - tickMsStart) < timeMs);
    }
}
