#include <stdint.h>
#include "stm32f10x.h"

#include "common/CommonMath.h"
#include "DevMotor.h"

#define PWM_FREQ                10000   // 10k

#define	motorLeftIn1Low()       GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define	motorLeftIn1High()      GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define	motorLeftIn2Low()       GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define	motorLeftIn2High()      GPIO_SetBits(GPIOC, GPIO_Pin_1)

#define	motorRightIn1Low()      GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define	motorRightIn1High()     GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define	motorRightIn2Low()      GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define	motorRightIn2High()     GPIO_SetBits(GPIOC, GPIO_Pin_3)

uint16_t devMotorMax;

void devMotorInit(void) {
    uint16_t period = SystemCoreClock / PWM_FREQ - 1;
    uint16_t prescaler = 0;

    devMotorMax = period;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    // Initialize Timer & PWM
    GPIO_InitTypeDef gpio_InitStruct;
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_InitStruct);

    TIM_TimeBaseInitTypeDef tim_TimeBaseStruct;
    tim_TimeBaseStruct.TIM_Period = period;
    tim_TimeBaseStruct.TIM_Prescaler = prescaler;
    tim_TimeBaseStruct.TIM_ClockDivision = 0;
    tim_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &tim_TimeBaseStruct);

    TIM_OCInitTypeDef tim_OCInitStruct;
    tim_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    tim_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    tim_OCInitStruct.TIM_Pulse = 0;
    tim_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &tim_OCInitStruct);

    tim_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    tim_OCInitStruct.TIM_Pulse = 0;
    TIM_OC4Init(TIM4, &tim_OCInitStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    // Initialize motor direction IO
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio_InitStruct);
}

void motorLeftForward() {
	motorLeftIn1High();
    motorLeftIn2Low();
}

void motorLeftBackward() {
	motorLeftIn1Low();
    motorLeftIn2High();
}

void motorLeftStop() {
	motorLeftIn1High();
    motorLeftIn2High();
}

void motorRightForward() {
	motorRightIn1Low();
    motorRightIn2High();
}

void motorRightBackward() {
	motorRightIn1High();
    motorRightIn2Low();
}

void motorRightStop() {
	motorRightIn1High();
    motorRightIn2High();
}

void devMotorSetSpeed(float left, float right) {
    // Looks like the duty cycle is actually reversed.
    float leftThrottle = 100.0 - frange(fabs(left), 0.0, 100.0);
    float rightThrottle = 100.0 - frange(fabs(right), 0.0, 100.0);

    uint16_t motorLeft = (devMotorMax + 1) * leftThrottle / 100.0 - 1;
    uint16_t motorRight = (devMotorMax + 1) * rightThrottle / 100.0 - 1;

    if (left > 0) {
        motorLeftForward();
    } else if (left < 0) {
        motorLeftBackward();
    } else if (left == 0) {
        motorLeftStop();
    }
    TIM_SetCompare3(TIM4, motorLeft);

    if (right > 0) {
        motorRightForward();
    } else if (right < 0) {
        motorRightBackward();
    } else if (right == 0) {
        motorRightStop();
    }
    TIM_SetCompare4(TIM4, motorRight);
}
