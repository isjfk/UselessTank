#include "ahrs.h"
#include "math.h"
#include "stdbool.h"
#include "motor.h"
#include "usart.h"
#include "delay.h"
#include "encoder.h"
#include "led.h"


#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

// ��ص�ѹ
extern float bat_volt;

// ҡ���ƶ����Ȳ�����PWMֵ
extern int joy_left_pwm, joy_right_pwm;

extern uint16_t cmd_status;


int Encoder_Left,Encoder_Right;             //�������������ֵ
int Moto1,Moto2;                       
int Voltage;                                //��Դ��ѹ
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ��Ƕȣ�ƽ������ʣ�ת�������

float Pitch = 0, Roll = 0, Yaw = 0;
short gyro[3], accel[3];
float Acceleration_Z;                       //Z����ٶ�

int Balance_Pwm, Velocity_Pwm, Turn_Pwm;

extern uint8_t rx_buffer[64];





void AHRS(void)
{
	float pitch,roll,yaw; 
	short aacx,aacy,aacz;	        
	short gyrox,gyroy,gyroz;     
	
	// ��ȡ����������Ϊ�����������ת��180�ȵģ�������Ҫ������һ��ȡ��
	Encoder_Left = Read_Encoder(2); 
	Encoder_Right = -Read_Encoder(8);                           
	
//	if(mpu_mpl_get_data(&pitch,&roll,&yaw)==0)
//	{
//		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������

//		Angle_Balance = roll;            // ��ȡ�����е�ROLL
//		Gyro_Balance = gyrox;            // ��ȡƽ����ٶȣ�����ָX������ʣ�
//		Gyro_Turn = gyroy;               // ��ȡת����ٶȣ�����ָY������ʣ�
//		Acceleration_Z = aacz;           // ��ȡZ����ٶ�

//		//printf("\r\n���ٶȣ� %8.2f%8.2f%8.2f    ", roll, yaw, pitch);
//	}
//	
//	Balance_Pwm = Balance(Angle_Balance,Gyro_Balance); 
//	Velocity_Pwm = velocity(Encoder_Left,Encoder_Right); 
//	Turn_Pwm = 0;  

	//printf("\r\n���ٶȣ� %8.2d%8.2f%8.2d%8.2d%8.2d%8.2d    ", Balance_Pwm, Gyro_Balance, Velocity_Pwm, Turn_Pwm, Encoder_Left, Encoder_Right);
	
//	Moto1=Balance_Pwm+Velocity_Pwm+Turn_Pwm + joy_left_pwm * 10;       // ���1PWM
//	Moto2=Balance_Pwm+Velocity_Pwm-Turn_Pwm + joy_right_pwm * 10;      // ���2PWM
	
	
	
	
	if(bat_volt < 7.4)
	{
		Set_Pwm(0, 0);  
		delay_ms(1000);
		Battery_Low_Sound();
	}
	else
	{
		Moto1 = joy_left_pwm * 10;       // ���1PWM
		Moto2 = joy_right_pwm * 10;      // ���2PWM
		Set_Pwm(-Moto1, -Moto2);    
	}
	
}





void ReadEncoder(void)
{
	Encoder_Left = (short)TIM2 -> CNT;
	TIM2 -> CNT = 0;
	
	Encoder_Right = (short)TIM4 -> CNT;
	TIM4 -> CNT = 0;
}


void Set_Pwm(int moto1, int moto2)
{
	if(moto1 < 0)
	{
		MotorDriver_L_Turn_Reverse();
		TIM_SetCompare3(TIM4, myabs(moto1)); 
	}
	else if(moto1 > 0)
	{
		MotorDriver_L_Turn_Forward();
		TIM_SetCompare3(TIM4, myabs(moto1)); 
	}
	else
	{
		MotorDriver_L_Turn_Stop();
		TIM_SetCompare3(TIM4, myabs(0)); 
	}
	
	
  
	
	if(moto2 < 0)
	{
		MotorDriver_R_Turn_Reverse();
		TIM_SetCompare4(TIM4, myabs(moto2));
	}
	else if(moto2 > 0)
	{
		MotorDriver_R_Turn_Forward();
		TIM_SetCompare4(TIM4, myabs(moto2));
	}
	else
	{
		MotorDriver_R_Turn_Stop();
		TIM_SetCompare4(TIM4, myabs(0)); 
	}
}





uint8_t Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle < -40 || angle > 40 || Voltage < 1110)  //��ѹ����11.1V �رյ��
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
	
	
	Bias = Angle - angle_adj;           // �Զ��ҳ�С������λ��
	
	
	
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
	
	
	
	
	
	balance = 500 * Bias + Gyro * 0.20;  // ƽ����Ƶ��PWM  PD���ƣ�500��Pϵ�� 0.20��Dϵ�� 
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


int myabs(int a)
{ 		   
	int temp;
	if(a < 0)
	{
		temp = -(a); 
	}
	else
	{
		temp = a;
	}
	
	
	if((7199 - temp) < 0)
	{
		temp = -(7199 - temp);
	}
	else
	{
		temp = 7199 - temp;
	}
	
	
	return temp;
}



