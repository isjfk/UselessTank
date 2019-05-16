#include <stdlib.h>
#include <math.h>

#include "system/SysTick.h"
#include "ps2.h"
#include "delay.h"
#include "servo.h"
#include "usart.h"
#include "stdbool.h"
#include "string.h"
#include "led.h"

#include "Tank.h"


extern bool balance_task;
extern u8 cmd_return[CMD_RETURN_SIZE];
extern u8 uart_receive_buf[UART_BUF_SIZE], uart1_get_ok, uart1_mode;

extern u8 group_do_ok;
extern int do_start_index, do_time, group_num_start, group_num_end, group_num_times;
extern servo duoji_doing[DJ_NUM];
/* Private define ------------------------------------------------------------*/


uint16_t cmd_status = 0;


u8 car_dw = 1;

u8 psx_buf[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 	//
const char *pre_cmd_set_red[PSX_BUTTON_NUM] = 
{
	"<PS2_RED01:#005P1400T2000!^$DST:5!>",	//L2						  
	"<PS2_RED02:#005P1850T2000!^$DST:5!>",	//R2						  
	"<PS2_RED03:#004P0600T2000!^$DST:4!>",	//L1						  
	"<PS2_RED04:#004P2400T2000!^$DST:4!>",	//R1			
	"<PS2_RED05:#002P2400T2000!^$DST:2!>",	//RU						  
	"<PS2_RED06:#003P0900T2000!^$DST:3!>",	//RR						  
	"<PS2_RED07:#002P0600T2000!^$DST:2!>",	//RD						  
	"<PS2_RED08:#003P2400T2000!^$DST:3!>",	//RL				
	"<PS2_RED09:$DGT:0-60,1!>",			//SE							  
	"<PS2_RED10:$DWD!>",					//AL						  
	"<PS2_RED11:$DWA!>",					//AR						  
	"<PS2_RED12:$DJR!>",					//ST			
	"<PS2_RED13:#001P2150T2000!^$DST:1!>",	//LU						  
	"<PS2_RED14:#000P0600T2000!^$DST:0!>",	//LR								  
	"<PS2_RED15:#001P0850T2000!^$DST:1!>",	//LD						  
	"<PS2_RED16:#000P2400T2000!^$DST:0!>",	//LL
};

const char *pre_cmd_set_grn[PSX_BUTTON_NUM] = 
{
	"<PS2_GRN01:$!>",	//L2						  
	"<PS2_GRN02:$!>",	//R2						  
	"<PS2_GRN03:$!>",	//L1						  
	"<PS2_GRN04:$!>",	//R1			
	"<PS2_GRN05:$!>",	//RU						  
	"<PS2_GRN06:$!>",	//RR						  
	"<PS2_GRN07:$!>",	//RD						  
	"<PS2_GRN08:$!>",	//RL				
	"<PS2_GRN09:$!>",			//SE							  
	"<PS2_GRN10:$!>",					//AL-NO						  
	"<PS2_GRN11:$!>",					//AR-NO						  
	"<PS2_GRN12:$!>",				//ST			
	"<PS2_GRN13:$!>",	//LU						  
	"<PS2_GRN14:$!>",	//LR								  
	"<PS2_GRN15:$!>",	//LD						  
	"<PS2_GRN16:$!>",	//LL						  
};



void handle_ps2(void)
{
	static u32 sysTickMsBak = 0;
	
	if(sysTickMs - sysTickMsBak < 20)
	{
		return;
	}
	
	sysTickMsBak = sysTickMs;
	psx_write_read(psx_buf);
	
	return;
}


/*
	$DST!
	$DST:x!
	$RST!
	$SADR:x!
	$CGP:%d-%d!
	$DEG:%d-%d!
	$DGS:x!
	$DGT:%d-%d,%d!
	$DCR:%d,%d!
	$DWA!
	$DWD!
	$DJR!
	$GETA!
*/

void parse_cmd(u8 *cmd)
{
	int pos, i, index, int1, int2, x, y, servo1, servo2;
	unsigned int temp[4];
	
	//uart1_send_str(cmd);
	if(pos = str_contain_str(uart_receive_buf, (u8 *)"$AP0:"), pos) 
	{
		i = 0;
		while(uart_receive_buf[i])
		{
			if((uart_receive_buf[i] == '$') && 
				 (uart_receive_buf[i + 1] == 'A') &&
			   (uart_receive_buf[i + 2] == 'P') &&
			   (uart_receive_buf[i + 3] == '0') &&
			   (uart_receive_buf[i + 4] == ':'))
			{
                uint8_t negative = 0;
				x = 0;
				
				i = i + 5;
                if (uart_receive_buf[i] == '-') {
                    negative = 1;
                    i++;
                }

				while(uart_receive_buf[i] && uart_receive_buf[i] != 'X')
				{
					x =  x * 10 + uart_receive_buf[i] - '0';
					i++;
				}

                if (negative) {
                    x = -x;
                }
			}
			else if(uart_receive_buf[i] == 'X') 
			{
                uint8_t negative = 0;
				y = 0;

				i++;

                if (uart_receive_buf[i] == '-') {
                    negative = 1;
                    i++;
                }

				while(uart_receive_buf[i] && uart_receive_buf[i] != 'Y')
				{
					y = y * 10 + uart_receive_buf[i] - '0';
					i++;
				}

                if (negative) {
                    y = -y;
                }
			}
			else if(uart_receive_buf[i] == 'Y') 
			{
				servo1 = 0;
				i++;
				while(uart_receive_buf[i] && uart_receive_buf[i] != 'A')
				{
					servo1 = servo1 * 10 + uart_receive_buf[i] - '0';
					i++;
				}
			}
			else if(uart_receive_buf[i] == 'A') 
			{
				servo2 = 0;
				i++;
				while(uart_receive_buf[i] && uart_receive_buf[i] != 'B')
				{
					servo2 = servo2 * 10 + uart_receive_buf[i] - '0';
					i++;
				}
			}
			else
			{
				i++;
			}
		}	
		
		
		cmd_status = 100;
		uart1_send_str(cmd);

        tankThrottleSet(y / 30000.0);
        tankYawSet(x / 30000.0);

		servo1 = servo1 * 8 + 600;
		
		duoji_doing[0].aim = servo1;
		duoji_doing[0].time = 100;
		duoji_doing[0].inc = (duoji_doing[0].aim -  duoji_doing[0].cur) / (duoji_doing[0].time/20.000);
		
		servo2 = servo2 * 9.5 + 600;
		
		duoji_doing[1].aim = servo2;
		duoji_doing[1].time = 100;
		duoji_doing[1].inc = (duoji_doing[1].aim -  duoji_doing[1].cur) / (duoji_doing[1].time/20.000);
		
		memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
		
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DST!"), pos)
	{
		group_do_ok  = 1;
		for(i=0;i<DJ_NUM;i++) 
		{
			duoji_doing[i].inc = 0;	
			duoji_doing[i].aim = duoji_doing[i].cur;
		}
		zx_uart_send_str((u8 *)"#255PDST!");
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DST:"), pos) 
	{
		if(sscanf((char *)cmd, "$DST:%d!", &index)) 
		{
			duoji_doing[index].inc = 0;	
			duoji_doing[index].aim = duoji_doing[index].cur;
			sprintf((char *)cmd_return, "#%03dPDST!\r\n", (int)index);
			zx_uart_send_str(cmd_return);
			memset(cmd_return, 0, sizeof(cmd_return));
		}
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$RST!"), pos) 
	{		

		//????? ?? W25Q64 ??? 8*1024/4 = 2048
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CGP:"), pos) 
	{		
		if(sscanf((char *)cmd, "$CGP:%d-%d!", &int1, &int2)) 
		{
			print_group(int1, int2);
		}
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DEG:"), pos) 
	{		
		if(sscanf((char *)cmd, "$DEG:%d-%d!", &int1, &int2)) 
		{
			//erase_sector(int1, int2);
		}
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DGS:"), pos) 
	{		
		if(sscanf((char *)cmd, "$DGS:%d!", &int1)) 
		{
			do_group_once(int1);
			group_do_ok = 1;
		}
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DGT:"), pos) 
	{		
		if(sscanf((char *)cmd, "$DGT:%d-%d,%d!", &group_num_start, &group_num_end, &group_num_times)) 
		{
			if(group_num_start != group_num_end) 
			{
				do_start_index = group_num_start;
			} 
			else 
			{
				do_group_once(group_num_start);
			}
			do_time = group_num_times;
			group_do_ok = 0;			
		}
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DCR:"), pos) 
	{		
		if(sscanf((char *)cmd, "$DCR:%d,%d!", &int1, &int2)) 
		{
			//car_pwm_set(int1, int2);	
		}
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DWA!"), pos) 
	{		
		car_dw--;
		if(car_dw == 0)car_dw = 1;
		BEEP_ON;
		delay_ms(100);
		BEEP_OFF;
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DWD!"), pos) 
	{		
		car_dw++;
		if(car_dw == 4)car_dw = 3;
		BEEP_ON;
		delay_ms(100);
		BEEP_OFF;
		
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_FARWARD!"), pos) 
	{		
		//car_pwm_set(1000, 1000);
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_BACKWARD!"), pos) 
	{		
		//car_pwm_set(-1000, -1000);
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_LEFT!"), pos) 
	{		
		//car_pwm_set(1000, -1000);
	}
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$CAR_RIGHT!"), pos) 
	{		
		//car_pwm_set(-1000, 1000);
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$DJR!"), pos) 
	{	
		zx_uart_send_str((u8 *)"#255P1500T2000!\r\n");
		for(i=0;i<DJ_NUM;i++) 
		{
			duoji_doing[i].aim = 1500;
			duoji_doing[i].time = 2000;
			duoji_doing[i].inc = (duoji_doing[i].aim -  duoji_doing[i].cur) / (duoji_doing[i].time/20.000);
		}
	} 
	else if(pos = str_contain_str(uart_receive_buf, (u8 *)"$GETA!"), pos) 
	{		
		uart1_send_str((u8 *)"AAA");
	} 
	else 
	{
		
	}
}

#define BUTTON_SELECT   (1<<0)
#define BUTTON_START    (1<<3)
void handleCfgButton(char newValue, char orgValue) {
    if (!(newValue & BUTTON_SELECT) && (orgValue & BUTTON_SELECT)) {
        tankPidDisableOnControlLow = !tankPidDisableOnControlLow;
        if (!tankPidDisableOnControlLow) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_START) && (orgValue & BUTTON_START)) {
        tankPidEnabled = !tankPidEnabled;
        if (tankPidEnabled) {
            beep(200, 90);
        }
    }
}

#define BUTTON_L1       (1<<2)
#define BUTTON_L2       (1<<0)
#define BUTTON_R1       (1<<3)
#define BUTTON_R2       (1<<1)
#define BUTTON_T        (1<<4)
#define BUTTON_X        (1<<6)
#define BUTTON_S        (1<<7)
#define BUTTON_O        (1<<5)
void handlePidButton(char newValue, char orgValue) {
    if (!(newValue & BUTTON_L1) && (orgValue & BUTTON_L1)) {
        tankPidSet.pid[0].kP += 10;
        if (fabs(tankPidSet.pid[0].kP) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_L2) && (orgValue & BUTTON_L2)) {
        tankPidSet.pid[0].kP -= 10;
        if (fabs(tankPidSet.pid[0].kP) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_R1) && (orgValue & BUTTON_R1)) {
        tankPidSet.pid[0].kI += 1;
        if (fabs(tankPidSet.pid[0].kI) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_R2) && (orgValue & BUTTON_R2)) {
        tankPidSet.pid[0].kI -= 1;
        if (fabs(tankPidSet.pid[0].kI) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_T) && (orgValue & BUTTON_T)) {
        tankPidSet.pid[0].kD += 1;
        if (fabs(tankPidSet.pid[0].kD) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_X) && (orgValue & BUTTON_X)) {
        tankPidSet.pid[0].kD -= 1;
        if (fabs(tankPidSet.pid[0].kD) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_S) && (orgValue & BUTTON_S)) {
        tankPidSet.pid[0].iLimit += 10;
        if (fabs(tankPidSet.pid[0].iLimit) < 0.1) {
            beep(200, 90);
        }
    }
    if (!(newValue & BUTTON_O) && (orgValue & BUTTON_O)) {
        tankPidSet.pid[0].iLimit -= 10;
        if (fabs(tankPidSet.pid[0].iLimit) < 0.1) {
            beep(200, 90);
        }
    }
}

void handle_button(void)
{
	// 保存本次手柄按键状态
	static unsigned char psx_button_bak[2] = {0};
	
	//uart1_send_str(psx_buf);
	
	// 如果按键状态没有变化
	if((psx_button_bak[0] == psx_buf[3]) && (psx_button_bak[1] == psx_buf[4])) 
	{				
	}
	else
	{
        handleCfgButton(psx_buf[3], psx_button_bak[0]);
        handlePidButton(psx_buf[4], psx_button_bak[1]);

		parse_psx_buf(psx_buf+3, psx_buf[1]);
		psx_button_bak[0] = psx_buf[3];
		psx_button_bak[1] = psx_buf[4];
	}
	
	
	
	if(psx_buf[1] == PS2_LED_GRN)
	{
//		joy_left_pwm = 0;
//		joy_right_pwm = 0;
	}
	else if(psx_buf[1] == PS2_LED_RED)
	{
		if((psx_buf[5] - 0x7F <= 10) && (psx_buf[5] - 0x7F >= -10))
		{
			psx_buf[5] = 0x7F;
		}
		
		if((psx_buf[6] - 0x7F <= 10) && (psx_buf[6] - 0x7F >= -10))
		{
			psx_buf[6] = 0x7F;
		}

        float throttle = (255 - psx_buf[6]) * (200.0 / 255.0) - 100;
        float yaw = (255 - psx_buf[5]) * (200.0 / 255.0) - 100;
        // Dead zone.
        if (fabs(throttle) < 10) {
            throttle = 0;
        }
        if (fabs(yaw) < 10) {
            yaw = 0;
        }

        // Limit yaw speed to easy control.
        yaw = yaw * 0.3;

        tankThrottleSet(throttle);
        tankYawSet(yaw);
	}

	return;
}


void parse_psx_buf(unsigned char *buf, unsigned char mode) 
{
	u8 i, pos = 0;
	static u16 bak=0xffff, temp, temp2;
	
	temp = (buf[0]<<8) + buf[1];
	
	if(bak != temp) 
	{
		temp2 = temp;
		temp &= bak;
		for(i=0;i<16;i++) 
		{
			if((1<<i) & temp) 
			{
			} 
			else 
			{
				if((1<<i) & bak) 
				{	//press											
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED) 
					{
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
					}
					else if(mode == PS2_LED_GRN) 
					{
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));
					} 
					else 
						continue;
					
					uart1_send_str(uart_receive_buf);
					
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) uart_receive_buf[pos-1] = '\0';
					if(str_contain_str(uart_receive_buf, (u8 *)"$")) 
					{
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf, (char *)cmd_return);
						uart1_get_ok = 1;
						uart1_mode = 1;
					} 
					else if(str_contain_str(uart_receive_buf, (u8 *)"#")) 
					{
						uart1_close();
						uart1_get_ok = 0;
						strcpy((char *)cmd_return, (char *)uart_receive_buf+11);
						strcpy((char *)uart_receive_buf,(char *) cmd_return);
						uart1_get_ok = 1;
						uart1_mode = 2;
					}
					
					//uart1_send_str(uart_receive_buf);
					bak = 0xffff;
				} 
				else 
				{//release		
					memset(uart_receive_buf, 0, sizeof(uart_receive_buf));					
					if(mode == PS2_LED_RED) 
					{
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_red[i], strlen(pre_cmd_set_red[i]));
					} 
					else if(mode == PS2_LED_GRN)
					{
						memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));
					} 
					else 
						continue;	
					
					pos = str_contain_str(uart_receive_buf, (u8 *)"^");
					if(pos) 
					{
						if(str_contain_str(uart_receive_buf+pos, (u8 *)"$")) 
						{
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							parse_cmd(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 1;
						}
						else if(str_contain_str(uart_receive_buf+pos, (u8 *)"#")) 
						{
							//uart1_close();
							//uart1_get_ok = 0;
							strcpy((char *)cmd_return, (char *)uart_receive_buf+pos);
							cmd_return[strlen((char *)cmd_return) - 1] = '\0';
							strcpy((char *)uart_receive_buf, (char *)cmd_return);
							do_action(uart_receive_buf);
							//uart1_get_ok = 1;
							//uart1_mode = 2;
						}
						//uart1_send_str(uart_receive_buf);
					}	
				}

			}
		}
		bak = temp2;
		BEEP_ON;
		delay_ms(10);
		BEEP_OFF;
	}	
	return;
}


void replace_char(u8*str, u8 ch1, u8 ch2)
{
	while(*str)
	{
		if(*str == ch1)
		{
			*str = ch2;
		} 
		str++;
	}
	return;
}


/*
	PS2_DAT PC14		IN
	PS2_CMD PA13		OUT
	PS2_ATT PA12		OUT
	PS2_CLK PA11 		OUT
*/
void psx_io_config(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure, GPIO_InitStructure2;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); 		//使能 PA 端口时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  //使能禁止JTAG
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; 					//配置 pin0-3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 			//IO 翻转 50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);  					//
	
	GPIO_InitStructure2.GPIO_Pin =GPIO_Pin_14;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure2);	
}


void PSX_init(void)
{
	psx_io_config();
	PS2_ATT(1);
	PS2_CMD(1);
	PS2_CLK(1);
	//PS2_DAT = 0;
	//PS2_ACK = 1;
}


unsigned char psx_transfer(unsigned char dat)
{
	unsigned char rd_data ,wt_data, i;
	wt_data = dat;
	rd_data = 0;
	//PS2_CLK(1);
	//tb_delay_us(10);
	for(i = 0;i < 8;i++)
	{
		PS2_CMD((wt_data & (0x01 << i)));
		PS2_CLK(1);
		delay_us(10);
		PS2_CLK(0);
		delay_us(10);
		PS2_CLK(1);
		//tb_delay_us(15);
		if(PS2_DAT) 
		{
			rd_data |= 0x01<<i;
		}
	}
	return rd_data;
}


void psx_write_read(unsigned char *get_buf)
{
	PS2_ATT(0);
	get_buf[0] = psx_transfer(START_CMD);
	get_buf[1] = psx_transfer(ASK_DAT_CMD);
	get_buf[2] = psx_transfer(get_buf[0]);
	get_buf[3] = psx_transfer(get_buf[0]);
	get_buf[4] = psx_transfer(get_buf[0]);
	get_buf[5] = psx_transfer(get_buf[0]);
	get_buf[6] = psx_transfer(get_buf[0]);
	get_buf[7] = psx_transfer(get_buf[0]);
	get_buf[8] = psx_transfer(get_buf[0]);	
	PS2_ATT(1);
	
	return;
}
