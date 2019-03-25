#ifndef _SERVO_H_
#define _SERVO_H_

#include "stm32f10x.h"


#define DJ_NUM 8

typedef struct
{
	uint8_t 	valid;//有效 TODO	
	uint16_t 	aim;	//执行目标
	uint16_t 	time;	//执行时间		
	float 		cur;	//当前值
	float 		inc;	//增量	
}servo;

extern u32 systick_ms;

u8 check_dj_state(void);
void handle_action(void);
void do_action(u8 *uart_receive_buf);
void do_group_once(int group_num);
int get_action_index(u8 *str);
void print_group(int start, int end);
void action_save(u8 *str);
void int_exchange(int *int1, int *int2);
void servo_set(u8 index, u8 level);
void TIM3_Int_Init(u16 arr,u16 psc);
float abs_float(float value);
void duoji_inc_handle(u8 index);
void TIM3_IRQHandler(void);
void servo_init(void);
void SysTick_Int_Init(void);
void SysTick_ms(void);
uint16_t str_contain_str(unsigned char *str, unsigned char *str2);

#endif

