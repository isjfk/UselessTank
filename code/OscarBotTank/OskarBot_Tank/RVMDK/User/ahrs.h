

#ifndef _AHRS_H_
#define _AHRS_H_

#include "stm32f10x.h"


int velocity(int encoder_left,int encoder_right);
int Balance(float Angle, float Gyro);
uint8_t Turn_Off(float angle, int voltage);
void Balancing_Loop(void);
int Turn(int encoder_left, int encoder_right, float gyro);
void Xianfu_Pwm(void);
void Parse(void);
void ReadEncoder(void);
void AHRS(void);


#endif // _AHRS_H_


