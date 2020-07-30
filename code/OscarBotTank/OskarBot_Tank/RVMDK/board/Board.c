#include <stdio.h>
#include <math.h>

#include "Board.h"
#include "common/CommonMath.c"
#include "system/SysIrq.h"
#include "system/SysDelay.h"
#include "system/SysTime.h"
#include "device/DevHx711.h"

float batteryLowVoltage = 10.7;         // For 3S LiPo battery.
int batteryLowStatus = 0;
float batteryVeryLowVoltage = 10.5;     // For 3S LiPo battery.
int batteryVeryLowStatus = 0;

int batteryAlarmStatus = 0;
uint32_t batteryAlarmPrevTimeMs = 0;

float boardBatteryVoltage = 0;
uint32_t boardBatteryVoltageTimeMs = 0;

DevButton powerButton;
DevButton stopButton;
bool shutdown = false;
bool powerOff = false;
uint32_t heartBeatTimeMs;

static uint32_t shutdownTimeMs = 0;
static uint32_t prevTimeMs = 0;
static bool foundHeartBeat = false;
static const uint32_t powerOffTimeoutMsWithHeartBeat =      10 * 1000;  // 10 seconds
static const uint32_t powerOffTimeoutMsWithoutHeartBeat =   60 * 1000;  // 60 seconds

void powerControlLoop(void);
void emergencyStopLoop(void);
void boardBatteryLoop(void);

void boardLoop(void) {
    devButtonLoop();
    // Update leash tension data from HX711.
    devHx711Loop();

    // Should be after devButtonLoop
    powerControlLoop();
    emergencyStopLoop();
    boardBatteryLoop();

    boardWdgReload();
}

void powerControlLoop(void) {
    static SysTimeLoop pwrLedLoop;

    if (!shutdown) {
        if (pdbIsPowerButtonUp()) {
            // Power button up, enter shutdown process
            sysTimeLoopStart(&pwrLedLoop, 500);
            pdbPowerLedOn();
            boardLedOn();
            boardBeepOn();

            shutdown = true;
            shutdownTimeMs = sysTimeCurrentMs();
            prevTimeMs = shutdownTimeMs;
        }
    } else {
        if (sysTimeLoopShouldEnter(&pwrLedLoop)) {
            pdbPowerLedToggle();
            boardLedToggle();
            boardBeepToggle();
        }

        if ((sysTimeCurrentMs() - prevTimeMs) > 500) {
            // Handle time jump when tank first sync with ROS
            shutdownTimeMs = sysTimeCurrentMs();
            foundHeartBeat = false;
        }

        if ((sysTimeCurrentMs() - heartBeatTimeMs) < 2000) {
            foundHeartBeat = true;
        }

        if (foundHeartBeat) {
            // Heart beat found
            uint32_t timeDiffMs = sysTimeCurrentMs() - heartBeatTimeMs;
            if (timeDiffMs > powerOffTimeoutMsWithHeartBeat) {
                powerOff = true;
            }
        } else {
            // No heart beat found, probably ROS is not started.
            uint32_t timeDiffMs = sysTimeCurrentMs() - shutdownTimeMs;
            if (timeDiffMs > powerOffTimeoutMsWithoutHeartBeat) {
                powerOff = true;
            }
        }

        if (powerOff) {
            pdbPowerOff();
        }

        prevTimeMs = sysTimeCurrentMs();
    }
}

void emergencyStopLoop(void) {
    static SysTimeLoop stopLedLoop;

    if (pdbIsStopButtonUp()) {
        pdbStopLedOff();
        sysTimeLoopStop(&stopLedLoop);
    } else {
        if (!sysTimeLoopIsStart(&stopLedLoop)) {
            sysTimeLoopStart(&stopLedLoop, 500);
        } else if (sysTimeLoopShouldEnter(&stopLedLoop)) {
            pdbStopLedToggle();
        }
    }
}

void boardWdgReload(void) {
    IWDG_ReloadCounter();
}

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

void alarmSystemError(void) {
    alarm(1000, 100);
    alarm(100, 100);
    alarm(100, 100);
    alarm(100, 1000);
}

void alarmGyroInitError(void) {
    alarm(1000, 100);
    alarm(100, 100);
    alarm(100, 1000);
}

void alarmGyroLoopError(void) {
    alarm(100, 100);
    alarm(100, 1000);
}

float boardGetBatteryVoltage(void) {
    return boardBatteryVoltage;
}

float boardBatteryVoltageFromAdc(void) {
	ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	return ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;
}

void boardMeasureBatteryVoltage(void) {
	float voltageCurrent = boardBatteryVoltageFromAdc();
    uint32_t sysTimeMsCurrent = sysTimeCurrentMs();
    uint32_t cycleTimeMs = sysTimeMsCurrent - boardBatteryVoltageTimeMs;
    uint32_t voltageRegulationCycleMs = 1000;

    if ((boardBatteryVoltageTimeMs == 0) || (cycleTimeMs > voltageRegulationCycleMs)) {
        boardBatteryVoltage = voltageCurrent;
    } else {
        float orgVoltagePercent = (voltageRegulationCycleMs - cycleTimeMs) / (float) voltageRegulationCycleMs;
        float currVoltagePercent = cycleTimeMs / (float) voltageRegulationCycleMs;
        boardBatteryVoltage = orgVoltagePercent * boardBatteryVoltage + currVoltagePercent * voltageCurrent;
    }

    boardBatteryVoltageTimeMs = sysTimeMsCurrent;
}

void detectBatteryLowStatus() {
    float batteryVoltage = boardGetBatteryVoltage();

    if (batteryVoltage < 0.5) {
        // Don't beep in case battery is not connected.
        batteryVeryLowStatus = 0;
        batteryLowStatus = 0;
    } else if (batteryVoltage < batteryVeryLowVoltage) {
        batteryVeryLowStatus = 1;
        batteryLowStatus = 1;
    } else if (batteryVoltage < batteryLowVoltage) {
        batteryLowStatus = 1;
    } else if (batteryVoltage == batteryLowVoltage) {
        batteryVeryLowStatus = 0;
    } else if (batteryVoltage > batteryLowVoltage) {
        batteryVeryLowStatus = 0;
        batteryLowStatus = 0;
    }
}

int calcAlarmIntervalTime() {
    float batteryVoltage = boardGetBatteryVoltage();

    float lowVoltageRange = fabs(batteryLowVoltage - batteryVeryLowVoltage);
    if (lowVoltageRange == 0) {
        // To avoid divide by zero.
        lowVoltageRange = 0.2;
    }

    float voltageDiff = frange(batteryVoltage - batteryVeryLowVoltage, 0, lowVoltageRange);
    int intervalTime = (voltageDiff / lowVoltageRange) * 400 + 100;

    return irange(intervalTime, 100, 500);
}

void batteryLowAlarmLoop() {
    if (boardIsBatteryLow()) {
        uint32_t currentSysTimeMs = sysTimeCurrentMs();
        int intervalTimeMs = boardIsBatteryVeryLow() ? 100 : calcAlarmIntervalTime();

        if ((currentSysTimeMs - batteryAlarmPrevTimeMs) >= (batteryAlarmStatus ? 100 : intervalTimeMs)) {
            batteryAlarmStatus = !batteryAlarmStatus;
            batteryAlarmPrevTimeMs = currentSysTimeMs;
        }

        if (batteryAlarmStatus) {
            boardBeepOn();
            boardLedOn();
        } else {
            boardBeepOff();
            boardLedOff();
        }
    } else {
        if (batteryAlarmStatus) {
            boardBeepOff();
            boardLedOff();
        }

        batteryAlarmPrevTimeMs = 0;
        batteryAlarmStatus = 0;
    }
}

void boardBatteryLoop(void) {
    boardMeasureBatteryVoltage();
    detectBatteryLowStatus();
    batteryLowAlarmLoop();
}

int8_t boardIsBatteryHealth(void) {
    return !boardIsBatteryLow();
}

int8_t boardIsBatteryLow(void) {
    return batteryLowStatus;
}

int8_t boardIsBatteryVeryLow(void) {
    return batteryVeryLowStatus;
}

void checkBatteryStatusOnInit(void) {
    // Alarm battery low in case battery is not connected only in initialize.
    boardMeasureBatteryVoltage();
    if (boardGetBatteryVoltage() < 0.5) {
        alarm(100, 500);
        alarm(100, 500);
        alarm(100, 500);
    }
}
