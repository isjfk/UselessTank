#include <stddef.h>
#include "stm32f10x.h"

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
 extern "C" {
#endif

#define boardLedOn()    GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define boardLedOff()   GPIO_SetBits(GPIOB, GPIO_Pin_14)

#define boardBeepOn()   GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define boardBeepOff()  GPIO_ResetBits(GPIOB, GPIO_Pin_13)

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
