#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "device/DevButton.h"
#include "device/DevGpio.h"

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
 extern "C" {
#endif

extern DevButton powerButton;
extern DevButton stopButton;
extern bool shutdown;
extern bool powerOff;

#define pdbPowerOn()            devGpioBitSetLow(GPIOC, GPIO_Pin_10)
#define pdbPowerOff()           devGpioBitSetHigh(GPIOC, GPIO_Pin_10)
#define pdbIsPowerButtonDown()  devButtonIsDown(&powerButton)
#define pdbIsPowerButtonUp()    devButtonIsUp(&powerButton)
#define pdbPowerLedOn()         devGpioBitSetLow(GPIOC, GPIO_Pin_12)
#define pdbPowerLedOff()        devGpioBitSetHigh(GPIOC, GPIO_Pin_12)
#define pdbPowerLedToggle()     devGpioBitToggle(GPIOC, GPIO_Pin_12)
#define pdbIsStopButtonDown()   devButtonIsDown(&stopButton)
#define pdbIsStopButtonUp()     devButtonIsUp(&stopButton)
#define pdbStopLedOn()          devGpioBitSetLow(GPIOD, GPIO_Pin_2)
#define pdbStopLedOff()         devGpioBitSetHigh(GPIOD, GPIO_Pin_2)
#define pdbStopLedToggle()      devGpioBitToggle(GPIOD, GPIO_Pin_2)
#define pdbIsRosPowerOn()       devGpioBitIsInputHigh(GPIOB, GPIO_Pin_10)
#define pdbIsRosPowerOff()      devGpioBitIsInputLow(GPIOB, GPIO_Pin_10)

#define boardBeepOn()           devGpioBitSetHigh(GPIOB, GPIO_Pin_13)
#define boardBeepOff()          devGpioBitSetLow(GPIOB, GPIO_Pin_13)
#define boardBeepToggle()       devGpioBitToggle(GPIOB, GPIO_Pin_13)

#define boardLedOn()            devGpioBitSetLow(GPIOB, GPIO_Pin_14)
#define boardLedOff()           devGpioBitSetHigh(GPIOB, GPIO_Pin_14)
#define boardLedToggle()        devGpioBitToggle(GPIOB, GPIO_Pin_14)

void boardDevInit(void);

void boardLoop(void);
void boardWdgReload(void);

void alarm(uint16_t onTime, uint16_t offTime);

void alarmSystemOk(void);
void alarmSystemError(void);
void alarmGyroInitError(void);
void alarmGyroLoopError(void);

void alarmRosOk(void);

void boardMeasureBatteryVoltage(void);
float boardGetBatteryVoltage(void);
int8_t boardIsBatteryHealth(void);
int8_t boardIsBatteryLow(void);
int8_t boardIsBatteryVeryLow(void);
void checkBatteryStatusOnInit(void);

static inline bool isShutdown(void) {
    return shutdown;
}

static inline bool isPowerOff(void) {
    return powerOff;
}

static inline bool setPowerOff(bool status) {
    bool orgStatus = powerOff;
    powerOff = status;
    return orgStatus;
}

void updateRosHeartBeatTimeMs(uint32_t timeMs);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
