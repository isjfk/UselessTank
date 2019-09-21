#include "stm32f10x.h"
#include "led.h"
#include "system/SysDelay.h"


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
	sysDelayMs(100);

	LED_OFF;
	sysDelayMs(100);
}

void beep(uint16_t onTime, uint16_t offTime) {
    BEEP_ON;
    LED_ON;
    sysDelayMs(onTime);

    BEEP_OFF;
    LED_OFF;
    sysDelayMs(offTime);
}

/**
	*	@brief		系统初始化成功后的声光提示
	*	@param		none
	*	@retval		none
	*/
void Sys_OK_Sound(void)
{
    beep(100, 100);
    beep(100, 100);
    beep(100, 100);
}

/**
	*	@brief		低电量声光提示
	*	@param		none
	*	@retval		none
	*/
void Battery_Low_Sound(void)
{
	beep(500, 500);
}

void beepSystemError(void) {
    beep(2000, 200);
    beep(200, 200);
    beep(200, 1000);
}

void beepGyroInitError(void) {
    beep(2000, 200);
    beep(200, 200);
    beep(200, 200);
    beep(200, 1000);
}

void beepGyroLoopError(void) {
    beep(2000, 200);
    beep(200, 200);
    beep(200, 200);
    beep(200, 200);
    beep(200, 1000);
}
