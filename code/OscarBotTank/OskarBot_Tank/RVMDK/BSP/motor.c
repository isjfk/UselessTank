#include "stm32f10x.h"
#include "motor.h"
#include "common/CommonMath.h"

#define	MOTOR_L_IN1_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_0))
#define	MOTOR_L_IN1_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_0))
#define	MOTOR_L_IN2_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_1))
#define	MOTOR_L_IN2_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_1))

#define	MOTOR_R_IN1_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_2))
#define	MOTOR_R_IN1_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_2))
#define	MOTOR_R_IN2_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_3))
#define	MOTOR_R_IN2_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_3))

uint16_t motorMax;

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void Motor_Init(u16 arr, u16 psc) {
    motorMax = arr;

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIO外设时钟使能
	                                                                     	
                                                                     	
   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9 ; //TIM4_CH3 TIM4_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision =0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure); 

  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;                         //设置待装入捕获比较寄存器的脉冲值
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);                     //输出极性:TIM输出比较极性高
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);            //根据TIM_OCInitStruct中指定的参数初始化外设TIMx


  TIM_CtrlPWMOutputs(TIM4,ENABLE);	      //MOE 主输出使能	
  TIM_ARRPreloadConfig(TIM4, ENABLE);     //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM4, ENABLE);                  //使能TIM4
	
	
	/*初始化PA.07端口为Out_PP模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void motorSet(int motorLeft, int motorRight) {
    motorLeft = irange(motorLeft, -motorMax, motorMax);
    motorRight = irange(motorRight, -motorMax, motorMax);

	if(motorLeft < 0) {
		MotorDriver_L_Turn_Reverse();
		TIM_SetCompare3(TIM4, motorMax - iabs(motorLeft)); 
	} else if(motorLeft > 0) {
		MotorDriver_L_Turn_Forward();
		TIM_SetCompare3(TIM4, motorMax - iabs(motorLeft)); 
	} else {
		MotorDriver_L_Turn_Stop();
		TIM_SetCompare3(TIM4, motorMax - iabs(0)); 
	}

	if(motorRight < 0) {
		MotorDriver_R_Turn_Reverse();
		TIM_SetCompare4(TIM4, motorMax - iabs(motorRight));
	} else if(motorRight > 0) {
		MotorDriver_R_Turn_Forward();
		TIM_SetCompare4(TIM4, motorMax - iabs(motorRight));
	} else {
		MotorDriver_R_Turn_Stop();
		TIM_SetCompare4(TIM4, motorMax - iabs(0)); 
	}
}

/**
	*	@brief		左轮电机正转
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_L_Turn_Forward(void)
{
	MOTOR_L_IN1_HIGH;
	MOTOR_L_IN2_LOW;
}

/**
	*	@brief		左轮电机反转
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_L_Turn_Reverse(void)
{
	MOTOR_L_IN1_LOW;
	MOTOR_L_IN2_HIGH;
}

/**
	*	@brief		左轮电机停转
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_L_Turn_Stop(void)
{
	MOTOR_L_IN1_HIGH;
	MOTOR_L_IN2_HIGH;
}

/**
	*	@brief		右轮电机正转
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_R_Turn_Forward(void)
{	
	MOTOR_R_IN1_LOW;
	MOTOR_R_IN2_HIGH;
}

/**
	*	@brief		右轮电机反转
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_R_Turn_Reverse(void)
{	
	MOTOR_R_IN1_HIGH;
	MOTOR_R_IN2_LOW;
}

/**
	*	@brief		右轮电机停转
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_R_Turn_Stop(void)
{	
	MOTOR_R_IN1_HIGH;
	MOTOR_R_IN2_HIGH;
}
