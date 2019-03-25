#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f10x.h"

void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);
void Detect_Volt(void);
void Init_Volt(void);



#endif // _ADC_H_