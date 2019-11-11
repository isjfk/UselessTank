#include "stm32f10x.h"

#ifndef __DEV_MOTOR_H
#define __DEV_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

void devMotorInit(void);

/**
 * Set motor by speed value.
 * 100.0: Set motor full speed forward
 * 50.0: Set motor half speed forward
 * 0: Set motor stop
 * -50.0: Set motor half speed backward
 * -100.0: Set motor full speed backward
 */
void devMotorSetSpeed(float left, float right);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_MOTOR_H */
