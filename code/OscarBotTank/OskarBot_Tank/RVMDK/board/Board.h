#include <stddef.h>
#include "stm32f10x.h"

#include "device/DevGpio.h"

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
 extern "C" {
#endif

#define pdbPowerOn()            GPIO_ResetBits(GPIOC, GPIO_Pin_10)
#define pdbPowerOff()           GPIO_SetBits(GPIOC, GPIO_Pin_10)
#define pdbIsPowerButtonDown()  GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11)
#define pdbIsPowerButtonUp()    (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11))
#define pdbPowerLedOn()         GPIO_ResetBits(GPIOC, GPIO_Pin_12)
#define pdbPowerLedOff()        GPIO_SetBits(GPIOC, GPIO_Pin_12)
#define pdbPowerLedToggle()     devGpioToggleOutputDataBit(GPIOC, GPIO_Pin_12)
#define pdbIsStopButtonDown()   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)
#define pdbIsStopButtonUp()     (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
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

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
