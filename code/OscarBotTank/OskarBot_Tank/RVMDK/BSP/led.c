#include "stm32f10x.h"
#include "led.h"
#include "delay.h"




/**
	*	@brief		LED和蜂鸣器初始化
	*	@param		none
	*	@retval		none
	*/
void Beep_Led_Init(void)
{
	GPIO_InitTypeDef					GPIO_InitStructure ;

	/*使能GPIOB外设时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/*初始化PB13和PB14端口为Out_PP模式*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	LED_ON;
	
	delay_ms(90);
	
	
	LED_OFF;
	
	delay_ms(90);
}


/**
	*	@brief		系统初始化成功后的声光提示
	*	@param		none
	*	@retval		none
	*/
void Sys_OK_Sound(void)
{
	BEEP_ON;
	
	LED_ON;
	
	delay_ms(90);
	
	BEEP_OFF;
	
	LED_OFF;
	
	delay_ms(90);
	
	BEEP_ON;
	
	LED_ON;
	
	delay_ms(90);
	
	BEEP_OFF;
	
	LED_OFF;
	
	delay_ms(90);
	
	BEEP_ON;
	
	LED_ON;
	
	delay_ms(90);
	
	BEEP_OFF;
	
	LED_OFF;
}


/**
	*	@brief		低电量声光提示
	*	@param		none
	*	@retval		none
	*/
void Battery_Low_Sound(void)
{
	BEEP_ON;
	
	LED_ON;
	
	delay_ms(500);
	
	BEEP_OFF;
	
	LED_OFF;
	
	delay_ms(500);
	
	BEEP_ON;
	
	LED_ON;
	
	delay_ms(500);
	
	BEEP_OFF;
	
	LED_OFF;
	
	delay_ms(500);
}


