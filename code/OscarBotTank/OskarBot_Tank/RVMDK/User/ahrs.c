#include <stdio.h>
#include <math.h>

#include "ahrs.h"
#include "stdbool.h"

#include "system/SysTick.h"
#include "board/Board.h"
#include "tank/Tank.h"
#include "device/DevMpu9250.h"
#include "device/DevMotor.h"
#include "device/DevHx711.h"

// 电池电压
extern float volatile bat_volt;

extern uint16_t cmd_status;


int Encoder_Left,Encoder_Right;             //两个电机编码器值
int motorL,motorR;
int Voltage;                                //电源电压
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡角度，平衡角速率，转向角速率

float Pitch = 0, Roll = 0, Yaw = 0;
short gyro[3], accel[3];
float Acceleration_Z;                       //Z轴加速度

int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

extern uint8_t rx_buffer[64];


void print_mpu9250_data() {
    static int prevTick = 0;

    int currTick = sysTickCurrentMs();
    if (currTick - prevTick > 100) {
        prevTick = currTick;

        int8_t accuracy;
        inv_time_t timestamp;
        float accel[3];
        float gyro[3];
        float compass[3];
        float heading[1];

        devMpu9250GetAccelFloat(accel, &accuracy, &timestamp);
        devMpu9250GetGyroFloat(gyro, &accuracy, &timestamp);
        devMpu9250GetCompassFloat(compass, &accuracy, &timestamp);
        devMpu9250GetHeadingFloat(heading, &accuracy, &timestamp);

        printf("gyro[%8.2f %8.2f %8.2f], accel[%8.2f %8.2f %8.2f], compass[%8.2f %8.2f %8.2f], heading[%8.2f] accuracy[%d]\r\n", gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], compass[0], compass[1], compass[2], heading[0], (int) accuracy);
    }
}

void print_tank_data() {
    int encoderLeft = devMotorGetLeftEncoder();
    int encoderRight = devMotorGetRightEncoder();
    //printf("throttle[%8.2f], throttleInput[%8.2f], yaw[%8.2f], yawInput[%8.2f]\r\n", tankThrottle, tankThrottleInput, tankYaw, tankYawInput);
    //printf("ps2thr[%8.2f], thr[%8.2f], ps2yaw[%8.2f], yaw[%8.2f]\r\n", ps2Throttle, tankThrottle, ps2Yaw, tankYaw);
    printf("encoderLeft[%6d], encoderRight[%6d]\r\n", encoderLeft, encoderRight);
}

void print_hx711_data() {
    int data = devHx711GetData();
    printf("HX711 hex[%08X] int[%11d] gram[%11d]\r\n", data, data, data/1500);
}

void AHRS(void)
{
    //print_mpu9250_data();
    //print_tank_data();
    //print_hx711_data();
}
