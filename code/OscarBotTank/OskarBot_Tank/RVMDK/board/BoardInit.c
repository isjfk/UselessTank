#include "BoardInit.h"
#include "Board.h"
#include "system/SysIrq.h"
#include "system/SysTick.h"
#include "system/SysDelay.h"
#include "device/DevCrc.h"
#include "device/DevMpu9250.h"
#include "device/DevUsart.h"
#include "device/DevMotor.h"
#include "device/DevHx711.h"

void boardBeepLedInit(void);
void boardTimerInit(void);
void boardUsartInit(void);
void boardMonitorInit(void);
void boardIwdgInit(void);

void boardInit(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    sysTickInit();

    boardBeepLedInit();
    //boardTimerInit();
    boardUsartInit();
    boardMonitorInit();

    devCrcInit();
    devMotorInit();
    devHx711Init();

    // Enable IRQ so SysTick can get correct value.
    // MPU9250 requires SysTick to initialize.
    ei();
    // Make sure SysTick run stablized.
    sysDelayMs(100);

    if (devMpu9250Init()) {
        alarmGyroInitError();
        NVIC_SystemReset();
    }

    checkBatteryStatusOnInit();
    boardIwdgInit();
}

void boardBeepLedInit(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);
	gpio_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	gpio_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_InitStruct);
}

void boardTimerInit(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseInitTypeDef tim_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&tim_TimeBaseInitStruct);
    tim_TimeBaseInitStruct.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;
    tim_TimeBaseInitStruct.TIM_Period = 1000 - 1;
    TIM_TimeBaseInit(TIM5, &tim_TimeBaseInitStruct);

    TIM_ARRPreloadConfig(TIM5, ENABLE);

    NVIC_InitTypeDef nvic_InitStruct;
    nvic_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_InitStruct.NVIC_IRQChannelSubPriority = 1;
    nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_InitStruct);

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

void boardUsart1GpioInit(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);

    /* Configure USART Tx as alternate function push-pull */
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_9;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_InitStruct);

    /* Configure USART Rx as input floating */
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_10;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_InitStruct);

    NVIC_InitTypeDef nvic_InitStruct;
    nvic_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_InitStruct.NVIC_IRQChannelSubPriority = 2;
    nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_InitStruct);
}

void boardUsart2GpioInit(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);

    /* Configure USART Tx as alternate function push-pull */
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_2;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_InitStruct);

    /* Configure USART Rx as input floating */
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_3;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_InitStruct);

    NVIC_InitTypeDef nvic_InitStruct;
    nvic_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_InitStruct.NVIC_IRQChannelSubPriority = 1;
    nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_InitStruct);
}

void boardUsart3GpioInit(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);

    /* Configure USART Tx as alternate function push-pull */
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_10;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio_InitStruct);

    /* Configure USART Rx as input floating */
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_11;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio_InitStruct);

    NVIC_InitTypeDef nvic_InitStruct;
    nvic_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    nvic_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_InitStruct.NVIC_IRQChannelSubPriority = 3;
    nvic_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_InitStruct);
}

void usartInit(USART_TypeDef* USARTx, uint32_t baudRate) {
    USART_DeInit(USARTx);

    USART_ClockInitTypeDef usart_ClockInitStruct;
    USART_ClockStructInit(&usart_ClockInitStruct);
    USART_ClockInit(USARTx, &usart_ClockInitStruct);

    USART_InitTypeDef usart_InitStruct;
    USART_StructInit(&usart_InitStruct);
    usart_InitStruct.USART_BaudRate = baudRate;
    USART_Init(USARTx, &usart_InitStruct);

    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
//    USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
//    USART_ITConfig(USARTx, USART_IT_PE, ENABLE);
//    USART_ITConfig(USARTx, USART_IT_ERR, ENABLE);

    USART_Cmd(USARTx, ENABLE);
}

void boardUsartInit(void) {
    boardUsart1GpioInit();
    usartInit(USART1, 115200);

    boardUsart2GpioInit();
    usartInit(USART2, 115200);

//    boardUsart3GpioInit();
//    usartInit(USART3, 115200);
//    USART_HalfDuplexCmd(USART3, ENABLE);
}

void boardAdcInit(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_ADC1, ENABLE );
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);
	gpio_InitStruct.GPIO_Pin = GPIO_Pin_4;
	gpio_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &gpio_InitStruct);

    ADC_DeInit(ADC1);

    ADC_InitTypeDef adc_InitStruct;
	adc_InitStruct.ADC_Mode = ADC_Mode_Independent;
	adc_InitStruct.ADC_ScanConvMode = DISABLE;
	adc_InitStruct.ADC_ContinuousConvMode = DISABLE;
	adc_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	adc_InitStruct.ADC_NbrOfChannel = 14;
	ADC_Init(ADC1, &adc_InitStruct);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

void boardMonitorInit(void) {
    boardAdcInit();
}

void boardIwdgInit(void) {
    uint32_t timeout = 3000;        // In ms.
    uint8_t prescaler = IWDG_Prescaler_64;
    uint16_t reload = timeout * 40000 / 1000 / (1 << (prescaler + 2));

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prescaler);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetReload(reload);
    IWDG_Enable();
}
