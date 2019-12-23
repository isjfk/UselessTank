#include <stdint.h>
#include "stm32f10x.h"

#include "common/CommonMath.h"
#include "DevMotor.h"

#define DEV_MOTOR_ESC_EXT           1

#define PWM_FREQ                    (10000)     // 10K
#define ENCODER_TIM_RELOAD          (0xFFFF)

#ifdef DEV_MOTOR_ESC_EXT
    #define motorLeftIn1Low()       GPIO_ResetBits(GPIOA, GPIO_Pin_8)
    #define motorLeftIn1High()      GPIO_SetBits(GPIOA, GPIO_Pin_8)
    #define motorLeftIn2Low()       GPIO_ResetBits(GPIOB, GPIO_Pin_15)
    #define motorLeftIn2High()      GPIO_SetBits(GPIOB, GPIO_Pin_15)
    #define motorLeftPwm(pwmValue)  TIM_SetCompare1(TIM3, pwmValue)

    #define motorRightIn1Low()      GPIO_ResetBits(GPIOC, GPIO_Pin_5)
    #define motorRightIn1High()     GPIO_SetBits(GPIOC, GPIO_Pin_5)
    #define motorRightIn2Low()      GPIO_ResetBits(GPIOB, GPIO_Pin_12)
    #define motorRightIn2High()     GPIO_SetBits(GPIOB, GPIO_Pin_12)
    #define motorRightPwm(pwmValue) TIM_SetCompare2(TIM3, pwmValue)
#else
    #define motorLeftIn1Low()       GPIO_ResetBits(GPIOC, GPIO_Pin_0)
    #define motorLeftIn1High()      GPIO_SetBits(GPIOC, GPIO_Pin_0)
    #define motorLeftIn2Low()       GPIO_ResetBits(GPIOC, GPIO_Pin_1)
    #define motorLeftIn2High()      GPIO_SetBits(GPIOC, GPIO_Pin_1)
    #define motorLeftPwm(pwmValue)  TIM_SetCompare3(TIM4, pwmValue)

    #define motorRightIn1Low()      GPIO_ResetBits(GPIOC, GPIO_Pin_2)
    #define motorRightIn1High()     GPIO_SetBits(GPIOC, GPIO_Pin_2)
    #define motorRightIn2Low()      GPIO_ResetBits(GPIOC, GPIO_Pin_3)
    #define motorRightIn2High()     GPIO_SetBits(GPIOC, GPIO_Pin_3)
    #define motorRightPwm(pwmValue) TIM_SetCompare4(TIM4, pwmValue)
#endif

uint16_t devMotorMax;

void devMotorCtrlInit(void);
void devMotorEncoderInit(void);

void motorLeftStop(void);
void motorRightStop(void);

void devMotorInit(void) {
    devMotorCtrlInit();
    devMotorEncoderInit();
}

#ifdef DEV_MOTOR_ESC_EXT
void devMotorCtrlInit(void) {
    uint16_t period = SystemCoreClock / PWM_FREQ - 1;
    uint16_t prescaler = 0;

    devMotorMax = period;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Timer 3 partial remap, Channel1->PB4, Channel2->PB5
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

    // Initialize Timer & PWM
    GPIO_InitTypeDef gpio_InitStruct;
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_InitStruct);

    TIM_TimeBaseInitTypeDef tim_TimeBaseStruct;
    tim_TimeBaseStruct.TIM_Period = period;
    tim_TimeBaseStruct.TIM_Prescaler = prescaler;
    tim_TimeBaseStruct.TIM_ClockDivision = 0;
    tim_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim_TimeBaseStruct);

    TIM_OCInitTypeDef tim_OCInitStruct;
    tim_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    tim_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    tim_OCInitStruct.TIM_Pulse = 0;
    tim_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &tim_OCInitStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM3, &tim_OCInitStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    // Initialize motor direction IO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    gpio_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    gpio_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &gpio_InitStruct);

    gpio_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio_InitStruct);

    gpio_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOC, &gpio_InitStruct);

    // Set initial motor state to stop
    motorLeftStop();
    motorRightStop();
}
#else
void devMotorCtrlInit(void) {
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
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
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
#endif

void devMotorLeftEncoderInit(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);
	gpio_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_InitStruct);

    TIM_TimeBaseInitTypeDef tim_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&tim_TimeBaseInitStruct);
    tim_TimeBaseInitStruct.TIM_Prescaler = 0x0;
    tim_TimeBaseInitStruct.TIM_Period = ENCODER_TIM_RELOAD;
    tim_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_TimeBaseInitStruct);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitTypeDef tim_ICInitStruct;
    TIM_ICStructInit(&tim_ICInitStruct);
    tim_ICInitStruct.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &tim_ICInitStruct);

    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

void devMotorRightEncoderInit(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    GPIO_StructInit(&gpio_InitStruct);
	gpio_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &gpio_InitStruct);

    TIM_TimeBaseInitTypeDef tim_TimeBaseInitStruct;
    TIM_TimeBaseStructInit(&tim_TimeBaseInitStruct);
    tim_TimeBaseInitStruct.TIM_Prescaler = 0x0;
    tim_TimeBaseInitStruct.TIM_Period = ENCODER_TIM_RELOAD;
    tim_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &tim_TimeBaseInitStruct);

    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitTypeDef tim_ICInitStruct;
    TIM_ICStructInit(&tim_ICInitStruct);
    tim_ICInitStruct.TIM_ICFilter = 10;
    TIM_ICInit(TIM8, &tim_ICInitStruct);

    TIM_SetCounter(TIM8, 0);
    TIM_Cmd(TIM8, ENABLE);
}

void devMotorEncoderInit(void) {
    devMotorLeftEncoderInit();
    devMotorRightEncoderInit();
}

void motorLeftForward() {
	motorLeftIn1Low();
    motorLeftIn2High();
}

void motorLeftBackward() {
	motorLeftIn1High();
    motorLeftIn2Low();
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
    motorLeftPwm(motorLeft);

    if (right > 0) {
        motorRightForward();
    } else if (right < 0) {
        motorRightBackward();
    } else if (right == 0) {
        motorRightStop();
    }
    motorRightPwm(motorRight);
}

uint32_t devMotorGetLeftEncoder(void) {
    // Motor left is mounted in reverse direction.
    return (-TIM_GetCounter(TIM2)) & 0xFFFF;
}

uint32_t devMotorGetRightEncoder(void) {
    return TIM_GetCounter(TIM8);
}
