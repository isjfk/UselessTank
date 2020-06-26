#include "SysTime.h"

int sysTimeLoopStart(SysTimeLoop *loop, uint32_t intervalTimeMs) {
    if (!loop) {
        return -1;
    }

    if (intervalTimeMs > 0) {
        loop->intervalTimeMs = intervalTimeMs;
    }
    loop->prevTimeMs = sysTickCurrentMs();

    return 0;
}

bool sysTimeLoopShouldEnter(SysTimeLoop *loop) {
    if (!sysTimeLoopIsStart(loop)) {
        return false;
    }

    uint32_t currTimeMs = sysTickCurrentMs();
    uint32_t intervalTimeMs = currTimeMs - loop->prevTimeMs;
    if (intervalTimeMs >= loop->intervalTimeMs) {
        loop->prevTimeMs = currTimeMs - (intervalTimeMs % loop->intervalTimeMs);
        return true;
    }

    return false;
}
