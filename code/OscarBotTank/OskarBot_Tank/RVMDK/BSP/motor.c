#include "stm32f10x.h"
#include "motor.h"


#define	MOTOR_L_IN1_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_0))
#define	MOTOR_L_IN1_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_0))
#define	MOTOR_L_IN2_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_1))
#define	MOTOR_L_IN2_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_1))

#define	MOTOR_R_IN1_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_2))
#define	MOTOR_R_IN1_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_2))
#define	MOTOR_R_IN2_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_3))
#define	MOTOR_R_IN2_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_3))





//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void Motor_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);// 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);  //ʹ��GPIO����ʱ��ʹ��
	                                                                     	
                                                                     	
   //���ø�����Ϊ�����������,���TIM1 CH1��PWM���岨��
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9 ; //TIM4_CH3 TIM4_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision =0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure); 

  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_Pulse = 0;                         //���ô�װ�벶��ȽϼĴ���������ֵ
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);                     //�������:TIM����Ƚϼ��Ը�
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);            //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx


  TIM_CtrlPWMOutputs(TIM4,ENABLE);	      //MOE �����ʹ��	
  TIM_ARRPreloadConfig(TIM4, ENABLE);     //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM4, ENABLE);                  //ʹ��TIM4
	
	
	/*��ʼ��PA.07�˿�ΪOut_PPģʽ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}



/**
	*	@brief		���ֵ����ת
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_L_Turn_Forward(void)
{
	MOTOR_L_IN1_LOW;
	MOTOR_L_IN2_HIGH;
}

/**
	*	@brief		���ֵ����ת
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_L_Turn_Reverse(void)
{
	MOTOR_L_IN1_HIGH;
	MOTOR_L_IN2_LOW;
}

/**
	*	@brief		���ֵ��ͣת
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_L_Turn_Stop(void)
{
	MOTOR_L_IN1_HIGH;
	MOTOR_L_IN2_HIGH;
}

/**
	*	@brief		���ֵ����ת
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_R_Turn_Forward(void)
{	
	MOTOR_R_IN1_HIGH;
	MOTOR_R_IN2_LOW;
}

/**
	*	@brief		���ֵ����ת
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_R_Turn_Reverse(void)
{	
	MOTOR_R_IN1_LOW;
	MOTOR_R_IN2_HIGH;
}


/**
	*	@brief		���ֵ��ͣת
	*	@param		none
	*	@retval		none
	*/
void	MotorDriver_R_Turn_Stop(void)
{	
	MOTOR_R_IN1_HIGH;
	MOTOR_R_IN2_HIGH;
}
