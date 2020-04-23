#include "SysTime.h"

int sysTimeIntervalInit(SysTimeInterval *interval, uint32_t intervalTimeMs) {
    if (!interval) {
        return -1;
    }

    interval->intervalTimeMs = intervalTimeMs;
    interval->prevTimeMs = sysTickCurrentMs();

    return 0;
}

bool sysTimeIsOnInterval(SysTimeInterval *interval) {
    uint32_t currTimeMs = sysTickCurrentMs();
    uint32_t intervalTimeMs = currTimeMs - interval->prevTimeMs;
    if (intervalTimeMs >= interval->intervalTimeMs) {
        interval->prevTimeMs = currTimeMs - (intervalTimeMs % interval->intervalTimeMs);
        return true;
    }
    return false;
}
