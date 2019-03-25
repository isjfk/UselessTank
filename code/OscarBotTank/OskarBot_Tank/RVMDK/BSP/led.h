#ifndef __LED_H
#define __LED_H

void Beep_Led_Init(void);
void Sys_OK_Sound(void);
void Battery_Low_Sound(void);

#define LED_ON   GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define LED_OFF  GPIO_SetBits(GPIOB, GPIO_Pin_14)

#define BEEP_ON   GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define BEEP_OFF  GPIO_ResetBits(GPIOB, GPIO_Pin_13)


#endif  //__LED_H
