#ifndef __MPU9250_BOARD_H
#define __MPU9250_BOARD_H

#include "delay.h"
#include "mpu9250.h"

#define Sensors_I2C_WriteRegister   MPU_Write_Len
#define Sensors_I2C_ReadRegister    MPU_Read_Len
#define __no_operation              __nop

inline int get_tick_count(unsigned long *count);
extern int get_tick_count(unsigned long *count);

#endif	/* __MPU9250_BOARD_H */
