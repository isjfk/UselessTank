#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f10x.h"

void MotorDriver_L_Turn_Forward(void);			//左轮电机正转
void MotorDriver_L_Turn_Reverse(void);			//左轮电机反转
void MotorDriver_L_Turn_Stop(void);         //左轮电机停转

void MotorDriver_R_Turn_Forward(void);			//右轮电机正转
void MotorDriver_R_Turn_Reverse(void);			//右轮电机反转
void MotorDriver_R_Turn_Stop(void);         //左轮电机停转

void Motor_Init(u16 arr,u16 psc);           //电机驱动外设初始化

#endif
