#ifndef __USART_H
#define __USART_H
#include "stdio.h"	


#include "stm32f10x.h"


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//All rights reserved
//********************************************************************************

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define UART_BUF_SIZE 1024


#define TB_USART1_COM 1
#define TB_USART2_COM 2
#define TB_USART3_COM 3

#define TB_USART_FLAG_ERR  0X0F
#define TB_USART_FLAG_RXNE 0X20
#define TB_USART_FLAG_TXE  0X80

#define tb_interrupt_open() {__enable_irq();}

#define uart1_open() 	{USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);}
#define uart1_close() 	{USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);}

#define uart2_open() 	{USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);}		
#define uart2_close() 	{USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);}		

#define uart3_open() 	{USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);}		
#define uart3_close() 	{USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);}		

void tb_usart_init(void);
void tb_usart1_init(u32 rate);
void tb_usart2_init(u32 rate);
void tb_usart3_init(u32 rate);

void tb_usart1_send_byte(u8 Data);
void tb_usart1_send_nbyte(u8 *Data, u16 size);
void tb_usart1_send_str(u8 *Data);

void tb_usart2_send_byte(u8 Data);
void tb_usart2_send_nbyte(u8 *Data, u16 size);
void tb_usart2_send_str(u8 *Data);

void tb_usart3_send_byte(u8 Data);
void tb_usart3_send_nbyte(u8 *Data, u16 size);
void tb_usart3_send_str(u8 *Data);


void uart1_send_str(u8 *str);
void uart1_send_nbyte(u8 *Data, u16 size);
void uart1_send_byte(u8 data);

void uart2_send_str(u8 *str);
void uart2_send_nbyte(u8 *Data, u16 size);
void uart2_send_byte(u8 data);

void zx_uart_send_str(u8 *str);
void uart3_send_str(u8 *str);
void uart3_send_nbyte(u8 *Data, u16 size);
void uart3_send_byte(u8 data);


void handle_uart(void);

void uart1_send_str(u8 *Data);
void zx_uart_send_str(u8 *str);
void tb_usart3_send_str(u8 *Data);

	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
#endif


