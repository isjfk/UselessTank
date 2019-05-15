#include "sys.h"
#include "servo.h"
#include "main.h"
#include "string.h"
#include "usart.h"
#include "ps2.h"

servo duoji_doing[DJ_NUM];
u8 duoji_index1;

u8 group_do_ok = 1;
int do_start_index, do_time, group_num_start, group_num_end, group_num_times;

u8 cmd_return[CMD_RETURN_SIZE];
extern u8 uart_receive_buf[UART_BUF_SIZE], uart1_get_ok, uart1_mode;

// Servo PWM timer
void TIM3_Int_Init(u16 arr,u16 psc) 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //??? TIM2 ??	
	//??? TIM2 ???
	TIM_TimeBaseStructure.TIM_Period = arr; //??????????????
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //?????????????
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ????
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  //???? TIM2
	TIM_ARRPreloadConfig(TIM3, DISABLE);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );  //???????
	
	//????? NVIC ??
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM2 ??
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //????? 0 ?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //???? 2 ?
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQ ?????
	NVIC_Init(&NVIC_InitStructure);  //???? NVIC ???
	TIM_Cmd(TIM3, ENABLE);  //??? TIM2
}


void TIM3_IRQHandler(void) 
{
	static u8 flag = 0;
	static u8 duoji_index1 = 0;
	int temp;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //?? TIM2 ????????
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update ); //?? TIM2 ??????
		
		if(duoji_index1 == 8) 
		{
			duoji_index1 = 0;
		}
		
		if(!flag) 
		{
			TIM3->ARR = ((unsigned int)(duoji_doing[duoji_index1].cur));
			servo_set(duoji_index1, 1);
			duoji_inc_handle(duoji_index1);
		} 
		else
		{
			temp = 2500 - (unsigned int)(duoji_doing[duoji_index1].cur);
			if(temp < 20)temp = 20;
			TIM3->ARR = temp;
			servo_set(duoji_index1, 0);
			duoji_index1 ++;
		}
		flag = !flag;
	}
} 

u8 check_dj_state(void) 
{
	int i;
	float	inc = 0;
	for(i=0;i<DJ_NUM;i++)
	{
		inc += duoji_doing[i].inc;
		if(inc)return 1;
	}
	return 0;
}


void int_exchange(int *int1, int *int2) 
{
	int int_temp;
	int_temp = *int1;
	*int1 = *int2;
	*int2 = int_temp;
}


void print_group(int start, int end)
{
	if(start > end) 
	{
		int_exchange(&start, &end);
	}
	
	for(;start<=end;start++) 
	{
		memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
		//w25x_read(uart_receive_buf, save_addr_sector*W25Q64_SECTOR_SIZE + start*ACTION_SIZE, ACTION_SIZE);
		uart1_send_str(uart_receive_buf);
		uart1_send_str((u8 *)"\r\n");
	}
}

void do_group_once(int group_num) 
{
	memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
	//w25x_read(uart_receive_buf, save_addr_sector*W25Q64_SECTOR_SIZE + group_num*ACTION_SIZE, ACTION_SIZE);
	do_action(uart_receive_buf);
	sprintf((char *)cmd_return, "do_group_once %d OK\r\n", group_num);
	uart1_send_str(cmd_return);
}


void handle_action(void)
{
	if(check_dj_state() == 0 && group_do_ok == 0)
	{
		do_group_once(do_start_index);
		
		if(group_num_start<group_num_end)
		{
			if(do_start_index == group_num_end)
			{
				do_start_index = group_num_start;
				if(group_num_times != 0)
				{
					do_time--;
					if(do_time == 0) 
					{
						group_do_ok = 1;
					}
				}
				return;
			}
			do_start_index++;
		} 
		else 
		{
			if(do_start_index == group_num_end) 
			{
				do_start_index = group_num_start;
				if(group_num_times != 0) 
				{
					do_time--;
					if(do_time == 0) 
					{
						group_do_ok = 1;
					}
				}
				return;
			}
			do_start_index--;
		}
	}
	
}

void action_save(u8 *str)
{
	int action_index = 0;
	action_index = get_action_index(str);
	if((action_index == -1) || str[6] != '#')
	{
		uart1_send_str((u8 *)"E");
		return;
	}
	//save_action_index_bak++;
	//if(action_index*ACTION_SIZE % W25Q64_SECTOR_SIZE == 0)w25x_erase_sector(action_index*ACTION_SIZE);
	replace_char(str, '<', '{');
	replace_char(str, '>', '}');
	//w25x_write(str, save_addr_sector*W25Q64_SECTOR_SIZE + action_index*ACTION_SIZE, strlen((char *)str) + 1);
	//uart1_send_str(uart_receive_buf);
	uart1_send_str((u8 *)"A");
	return;	
}

void do_action(u8 *uart_receive_buf)
{
	u16 index, pwm, time,i;
	zx_uart_send_str(uart_receive_buf);
	i = 0;
	while(uart_receive_buf[i])
	{
		if(uart_receive_buf[i] == '#')
		{
			index = 0;
			i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != 'P')
			{
				index = index*10 + uart_receive_buf[i]-'0';
				i++;
			}
		}
		else if(uart_receive_buf[i] == 'P') 
		{
			pwm = 0;
			i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != 'T')
			{
				pwm = pwm*10 + uart_receive_buf[i] - '0';
				i++;
			}
		} 
		else if(uart_receive_buf[i] == 'T') 
		{
			time = 0;
			i++;
			while(uart_receive_buf[i] && uart_receive_buf[i] != '!') 
			{
				time = time*10 + uart_receive_buf[i]-'0';
				i++;
			}
			
			if(index < DJ_NUM && (pwm<=2500)&& (pwm>=500) && (time<=10000)) 
			{
				//duoji_doing[index].inc = 0;
				if(duoji_doing[index].cur == pwm)pwm += 0.1;
				if(time < 20)time = 20;
				duoji_doing[index].aim = pwm;
				duoji_doing[index].time = time;
				duoji_doing[index].inc = (duoji_doing[index].aim -  duoji_doing[index].cur) / (duoji_doing[index].time/20.000);
			}
			
			//sprintf(cmd_return, "#%dP%dT%d!\r\n", index, pwm, time, duoji_doing[index].inc);
			//uart1_send_str(cmd_return);
			
		}
		else
		{
			i++;
		}
	}	
}




int get_action_index(u8 *str) 
{
	u16 index = 0;
	//uart_send_str(str);
	while(*str) 
	{
		if(*str == 'G') 
		{
			str++;
			while((*str != '#') && (*str != '$')) 
			{
				index = index*10 + *str-'0';
				str++;	
			}
			return index;
		} 
		else 
		{
			str++;
		}
	}
	return -1;
}



void duoji_inc_handle(u8 index) 
{	
	if(duoji_doing[index].inc != 0) 
	{
		if(abs_float(duoji_doing[index].aim - duoji_doing[index].cur) <= abs_float(duoji_doing[index].inc + duoji_doing[index].inc)) 
		{
			duoji_doing[index].cur = duoji_doing[index].aim;
			duoji_doing[index].inc = 0;
		} 
		else 
		{
			duoji_doing[index].cur += duoji_doing[index].inc;
		}
	}
}



void servo_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	u8 i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_12|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	

	//舵机控制初始化
	for(i=0;i<DJ_NUM;i++) 
	{
		duoji_doing[i].cur = 1500;
		duoji_doing[i].inc = 0;
	}
	
//	duoji_doing[0].cur = 1500;
//	duoji_doing[0].inc = 0;
//	duoji_doing[1].cur = 1900;
//	duoji_doing[1].inc = 0;
//	duoji_doing[2].cur = 800;
//	duoji_doing[2].inc = 0;
//	duoji_doing[3].cur = 2100;
//	duoji_doing[3].inc = 0;
//	duoji_doing[4].cur = 1500;
//	duoji_doing[4].inc = 0;
//	duoji_doing[5].cur = 1000;
//	duoji_doing[5].inc = 0;

	duoji_index1 = 0;

	//servor tim init
	TIM3_Int_Init(20000, 71);
}




void gpioA_pin_set(unsigned char pin, unsigned char level) 
{
	if(level) 
	{
		GPIO_SetBits(GPIOA,1 << pin);
	} 
	else 
	{
		GPIO_ResetBits(GPIOA,1 << pin);
	}
}

void gpioB_pin_set(unsigned char pin, unsigned char level)
{
	if(level) 
	{
		GPIO_SetBits(GPIOB,1 << pin);
	} 
	else
	{
		GPIO_ResetBits(GPIOB,1 << pin);
	}
}


void gpioC_pin_set(unsigned char pin, unsigned char level) 
{
	if(level)
	{
		GPIO_SetBits(GPIOC,1 << pin);
	} 
	else
	{
		GPIO_ResetBits(GPIOC,1 << pin);
	}
}



void servo_set(u8 index, u8 level)
{
	switch(index) 
	{
		case 0:gpioB_pin_set(12, level);break;
		case 1:gpioC_pin_set(5, level);break;
		case 2:gpioB_pin_set(15, level);break;
		case 3:gpioA_pin_set(8, level);break;
		case 4:gpioB_pin_set(5, level);break;
		case 5:gpioB_pin_set(4, level);break;
		case 6:gpioB_pin_set(3, level);break;
		case 7:gpioA_pin_set(15, level);break;
		default:break;
	}
}





float abs_float(float value) 
{
	if(value>0) 
	{
		return value;
	}
	return (-value);
}


uint16_t str_contain_str(unsigned char *str, unsigned char *str2) 
{
	unsigned char *str_temp, *str_temp2;
	str_temp = str;
	str_temp2 = str2;
	while(*str_temp) 
	{
		if(*str_temp == *str_temp2) 
		{
			while(*str_temp2) 
			{
				if(*str_temp++ != *str_temp2++) 
				{
					str_temp = str_temp - (str_temp2-str2) + 1;
					str_temp2 = str2;
					break;
				}	
			}
			
			if(!*str_temp2)
			{
				return (str_temp-str);
			}
			
		}
		else 
		{
			str_temp++;
		}
	}
	
	return 0;
}
