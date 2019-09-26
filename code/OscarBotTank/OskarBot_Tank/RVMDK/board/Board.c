#include <stdio.h>

#include "Board.h"
#include "system/SysTick.h"
#include "system/SysDelay.h"

float batteryHealthVoltage = 10.2;      // For 3S LiPo battery.
float boardBatteryVoltage = 0;
uint32_t boardBatteryVoltageSysTickMs = 0;

void boardLoop(void) {
    boardMeasureBatteryVoltage();
    if (!boardIsBatteryHealth()) {
        alarmBatteryLow();
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

int8_t boardIsBatteryLow(void) {
    float batteryVoltage = boardGetBatteryVoltage();

    // Don't consider battery low in case battery is not connected.
    return (batteryVoltage > 0.5) && (batteryVoltage < batteryHealthVoltage);
}

int8_t boardIsBatteryHealth(void) {
    return !boardIsBatteryLow();
}
