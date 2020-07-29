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

#define pdbPowerOn()            GPIO_ResetBits(GPIOC, GPIO_Pin_10)
#define pdbPowerOff()           GPIO_SetBits(GPIOC, GPIO_Pin_10)
#define pdbIsPowerButtonDown()  devButtonIsDown(&powerButton)
#define pdbIsPowerButtonUp()    devButtonIsUp(&powerButton)
#define pdbPowerLedOn()         GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define pdbPowerLedOff()        GPIO_SetBits(GPIOC, GPIO_Pin_12)
#define pdbPowerLedToggle()     devGpioToggleOutputDataBit(GPIOC, GPIO_Pin_12)
#define pdbIsStopButtonDown()   devButtonIsDown(&stopButton)
#define pdbIsStopButtonUp()     devButtonIsUp(&stopButton)
#define pdbStopLedOn()          GPIO_ResetBits(GPIOD, GPIO_Pin_2)
#define pdbStopLedOff()         GPIO_SetBits(GPIOD, GPIO_Pin_2)
#define pdbStopLedToggle()      devGpioToggleOutputDataBit(GPIOD, GPIO_Pin_2)

#define boardLedOn()            GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define boardLedOff()           GPIO_SetBits(GPIOB, GPIO_Pin_14)

#define boardBeepOn()           GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define boardBeepOff()          GPIO_ResetBits(GPIOB, GPIO_Pin_13)

void boardLoop(void);
void boardWdgReload(void);

void alarm(uint16_t onTime, uint16_t offTime);

void alarmSystemOk(void);
void alarmSystemError(void);
void alarmGyroInitError(void);
void alarmGyroLoopError(void);

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

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
