#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f10x.h"

void MotorDriver_L_Turn_Forward(void);			//���ֵ����ת
void MotorDriver_L_Turn_Reverse(void);			//���ֵ����ת
void MotorDriver_L_Turn_Stop(void);         //���ֵ��ͣת

void MotorDriver_R_Turn_Forward(void);			//���ֵ����ת
void MotorDriver_R_Turn_Reverse(void);			//���ֵ����ת
void MotorDriver_R_Turn_Stop(void);         //���ֵ��ͣת

void Motor_Init(u16 arr,u16 psc);           //������������ʼ��

#endif
