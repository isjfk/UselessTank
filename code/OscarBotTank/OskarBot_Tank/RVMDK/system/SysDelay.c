#include "SysDelay.h"
#include "SysTick.h"

void sysDelayUs(uint32_t timeUs) {
    uint32_t tickValPrev = SysTick->VAL;
    uint32_t tickValCurr = tickValPrev;
    uint32_t tickVal = 0;
    uint32_t delayUs = 0;

    while (delayUs < timeUs) {
        tickValCurr = SysTick->VAL;

        tickVal += (tickValPrev - tickValCurr) + ((tickValPrev < tickValCurr) ? sysTickValMax : 0);
        delayUs += tickVal / sysTickValPerUs;
        tickVal = tickVal % sysTickValPerUs;

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
