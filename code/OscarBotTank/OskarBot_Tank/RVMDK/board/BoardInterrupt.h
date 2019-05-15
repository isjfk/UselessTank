#include "stm32f10x.h"

#ifndef __BOARD_INTERRUPT_H
#define __BOARD_INTERRUPT_H

#ifdef __cplusplus
 extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM4_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_INTERRUPT_H */
