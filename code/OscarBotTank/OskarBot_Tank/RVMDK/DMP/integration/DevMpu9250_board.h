#ifndef __MPU9250_BOARD_H
#define __MPU9250_BOARD_H

#include "system/SysTick.h"
#include "system/SysDelay.h"
#include "i2c.h"

#define Sensors_I2C_WriteRegister   MPU_Write_Len
#define Sensors_I2C_ReadRegister    MPU_Read_Len
#define __no_operation              __nop

static inline int get_tick_count(unsigned long *count)
{
    *count = sysTickMs;
    return 0;
}

// FIXME:
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);

#endif	/* __MPU9250_BOARD_H */
