#include <stdio.h>

#include "Board.h"
#include "Tank.h"
#include "system/SysTick.h"
#include "device/DevMpu9250.h"
#include "service/PID.h"

#include "led.h"    // FIXME:

void boardTimerInit();

void boardInit(void) {
    SystemInit();

    sysTickInit();
    Beep_Led_Init();

    tankInit();

    if (devMpu9250Init()) {
        beepGyroInitError();
    }

    boardTimerInit();
}

void boardTimerInit() {
    NVIC_InitTypeDef nvic_init_struct;
    TIM_TimeBaseInitTypeDef tim_time_base_init_struct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    nvic_init_struct.NVIC_IRQChannel = TIM5_IRQn;
    nvic_init_struct.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init_struct.NVIC_IRQChannelSubPriority = 1;
    nvic_init_struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_struct);

    TIM_TimeBaseStructInit(&tim_time_base_init_struct);
    tim_time_base_init_struct.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;
    tim_time_base_init_struct.TIM_Period = 1000 - 1;

    TIM_TimeBaseInit(TIM5, &tim_time_base_init_struct);
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}

void TIM5_IRQHandler(void) {
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == RESET) {
        return;
    }
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

    if (devMpu9250Loop()) {
        beepGyroLoopError();
    }

    if (devMpu9250GyroUpdated) {
        tankPidLoop();
    }
}
