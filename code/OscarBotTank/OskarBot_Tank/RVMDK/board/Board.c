#include "Board.h"
#include "system/SysDelay.h"

void alarm(uint16_t onTime, uint16_t offTime) {
    boardBeepOn();
    boardLedOn();
    sysDelayMs(onTime);

    boardBeepOff();
    boardLedOff();
    sysDelayMs(offTime);
}

void alarmSystemOk(void) {
    alarm(100, 100);
    alarm(100, 100);
    alarm(100, 100);
}

void alarmBatteryLow(void) {
    alarm(500, 500);
}

void alarmSystemError(void) {
    alarm(2000, 200);
    alarm(200, 200);
    alarm(200, 1000);
}

void alarmGyroInitError(void) {
    alarm(2000, 200);
    alarm(200, 200);
    alarm(200, 200);
    alarm(200, 1000);
}

void alarmGyroLoopError(void) {
    alarm(2000, 200);
    alarm(200, 200);
    alarm(200, 200);
    alarm(200, 200);
    alarm(200, 1000);
}
