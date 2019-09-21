#include <math.h>

#include "ahrs.h"
#include "stdbool.h"
#include "motor.h"
#include "usart.h"
#include "encoder.h"
#include "ps2.h"

#include "system/SysTick.h"
#include "device/DevMpu9250.h"
#include "board/Board.h"
#include "tank/Tank.h"

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
    if (currTick - prevTick > 500) {
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
    //printf("throttle[%8.2f], throttleFixed[%8.2f], yaw[%8.2f], yawFixed[%8.2f]\r\n", tankThrottle, tankThrottleFixed, tankYaw, tankYawFixed);
    printf("ps2thr[%8.2f], thr[%8.2f], ps2yaw[%8.2f], yaw[%8.2f]\r\n", ps2Throttle, tankThrottle, ps2Yaw, tankYaw);
}

void AHRS(void)
{
	// 读取编码器，因为两个电机的旋转了180度的，所以需要对其中一个取反
	//Encoder_Left = Read_Encoder(2); 
	//Encoder_Right = -Read_Encoder(8);                           

	//Balance_Pwm = Balance(Angle_Balance,Gyro_Balance); 
	//Velocity_Pwm = velocity(Encoder_Left,Encoder_Right); 
	//Turn_Pwm = 0;  

	//printf("\r\n加速度： %8.2d%8.2f%8.2d%8.2d%8.2d%8.2d    ", Balance_Pwm, Gyro_Balance, Velocity_Pwm, Turn_Pwm, Encoder_Left, Encoder_Right);

	//Moto1=Balance_Pwm+Velocity_Pwm+Turn_Pwm + joy_left_pwm * 10;       // 电机1PWM
	//Moto2=-Balance_Pwm+Velocity_Pwm-Turn_Pwm + joy_right_pwm * 10;      // 电机2PWM
    
    print_mpu9250_data();
    //print_tank_data();

    if (bat_volt < 10.2) {              // low battery voltage for 3S
        motorSet(0, 0);
        alarmBatteryLow();
    } else {
        motorL = tankControlRange(tankThrottleGet() - tankYawGet()) * 71.99;
        motorR = tankControlRange(tankThrottleGet() + tankYawGet()) * 71.99;

        motorSet(motorL, motorR);
    }
}

void ReadEncoder(void)
{
	Encoder_Left = (short)TIM2 -> CNT;
	TIM2 -> CNT = 0;
	
	Encoder_Right = (short)TIM4 -> CNT;
	TIM4 -> CNT = 0;
}

uint8_t Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle < -40 || angle > 40 || Voltage < 1110)  //电压低于11.1V 关闭电机
	{	                                                                
		temp=1;                                                         
	}
	else
	{
		temp = 0;
	}
	
	return temp;			
}



int Balance(float Angle, float Gyro)
{
	static int i = 0;
	static float last_angle = 0.001, angle_sum = 0.001, angle_adj = 0;
	float Bias;
	int balance;
	
	
	Bias = Angle - angle_adj;           // 自动找出小车重心位置
	
	
	
	if(cmd_status > 0)
	{
		cmd_status--;
	}
	
	
	if(cmd_status == 0)
	{
		if( ((last_angle >= 0) && (Angle >= 0)) || ((last_angle <= 0) && (Angle <= 0)) )
		{
			i++;
			angle_sum = angle_sum + Angle;
			last_angle = Angle;
		}
		else
		{
			i = 0;
			last_angle = 0.001;
			angle_sum = 0.001;
		}
		
		
		if(i > 50)
		{
			angle_adj = angle_sum/50;
			
			i = 0;
			last_angle = 0.01;
			angle_sum = 0.01;
		}
		
	}
	
	
	
	
	
	balance = 500 * Bias + Gyro * 0.20;  // 平衡控制电机PWM  PD控制：500是P系数 0.20是D系数 
	return balance;
}




int velocity(int encoder_left,int encoder_right)
{  
	static int Velocity, Encoder_Least, Encoder, Movement;
	static int Encoder_Integral;
	
	Movement=0;
	Encoder_Least =(Encoder_Left+Encoder_Right)-0;  
	Encoder *= 0.8;		                            
	Encoder += Encoder_Least*0.2;	              

	
	if(Turn_Off(Angle_Balance, Voltage) == 0)   
	{	
		Encoder_Integral += Encoder;               
		Encoder_Integral = Encoder_Integral - Movement; 
	}
	
	if(Encoder_Integral > 360000)
	{
		Encoder_Integral = 360000;
	}
	
	if(Encoder_Integral < -360000)
	{
		Encoder_Integral = -360000; 
	}
	
	Velocity = Encoder * 130 + Encoder_Integral * 0.4;  
	
	return Velocity;
}
