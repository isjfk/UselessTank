#include <stdio.h>
#include <math.h>

#include "Board.h"
#include "common/CommonMath.c"
#include "system/SysTick.h"
#include "system/SysIrq.h"
#include "system/SysDelay.h"
#include "device/DevHx711.h"

float batteryLowVoltage = 10.7;         // For 3S LiPo battery.
int batteryLowStatus = 0;
float batteryVeryLowVoltage = 10.5;     // For 3S LiPo battery.
int batteryVeryLowStatus = 0;

int batteryAlarmStatus = 0;
uint32_t batteryAlarmPrevSysTickMs = 0;

float boardBatteryVoltage = 0;
uint32_t boardBatteryVoltageSysTickMs = 0;

void detectBatteryLowStatus(void);
void batteryLowAlarmLoop(void);

void boardLoop(void) {
    boardMeasureBatteryVoltage();
    detectBatteryLowStatus();
    batteryLowAlarmLoop();

    // Update leash tension data from HX711.
    devHx711Loop();
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
        uint32_t currentSysTickMs = sysTickCurrentMs();
        int intervalTimeMs = boardIsBatteryVeryLow() ? 100 : calcAlarmIntervalTime();

        if ((currentSysTickMs - batteryAlarmPrevSysTickMs) >= (batteryAlarmStatus ? 100 : intervalTimeMs)) {
            batteryAlarmStatus = !batteryAlarmStatus;
            batteryAlarmPrevSysTickMs = currentSysTickMs;
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

        batteryAlarmPrevSysTickMs = 0;
        batteryAlarmStatus = 0;
    }
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
    uint32_t sysTickMsCurrent = sysTickCurrentMs();
    uint32_t cycleTimeMs = sysTickMsCurrent - boardBatteryVoltageSysTickMs;
    uint32_t voltageRegulationCycleMs = 1000;

    if ((boardBatteryVoltageSysTickMs == 0) || (cycleTimeMs > voltageRegulationCycleMs)) {
        boardBatteryVoltage = voltageCurrent;
    } else {
        float orgVoltagePercent = (voltageRegulationCycleMs - cycleTimeMs) / (float) voltageRegulationCycleMs;
        float currVoltagePercent = cycleTimeMs / (float) voltageRegulationCycleMs;
        boardBatteryVoltage = orgVoltagePercent * boardBatteryVoltage + currVoltagePercent * voltageCurrent;
    }

    boardBatteryVoltageSysTickMs = sysTickMsCurrent;
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

int8_t boardIsBatteryHealth(void) {
    return !boardIsBatteryLow();
}

int8_t boardIsBatteryLow(void) {
    return batteryLowStatus;
}

int8_t boardIsBatteryVeryLow(void) {
    return batteryVeryLowStatus;
}

void alarmBatteryLow(void) {
    alarm(100, 500);
}

void checkBatteryStatusOnInit(void) {
    // Alarm battery low in case battery is not connected only in initialize.
    boardMeasureBatteryVoltage();
    if (boardGetBatteryVoltage() < 0.5) {
        alarmBatteryLow();
        alarmBatteryLow();
        alarmBatteryLow();
    }
}
