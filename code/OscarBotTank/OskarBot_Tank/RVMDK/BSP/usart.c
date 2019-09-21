#include "sys.h"
#include "usart.h"
#include "main.h"
#include "ps2.h"
#include "servo.h"
#include "led.h"


/*
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0)
		;//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
*/

 





u8 uart_receive_buf[UART_BUF_SIZE], uart1_get_ok, uart1_mode;





void tb_usart_init(void) 
{
	tb_usart1_init(115200);
	uart1_open();
	
	tb_usart2_init(115200);
	uart2_open();
	
	tb_usart3_init(115200);
	uart3_open();
	
	tb_interrupt_open();
	return;
}

void tb_usart1_init(u32 rate) 
{  
	GPIO_InitTypeDef GPIO_InitStructure;  
	USART_InitTypeDef USART_InitStructure; 
	USART_ClockInitTypeDef USART_ClockInitStructure; 	
	NVIC_InitTypeDef NVIC_InitStructure;  


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);  
	USART_DeInit(USART1);  
	/* Configure USART Tx as alternate function push-pull */  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	/* Configure USART Rx as input floating */  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	USART_InitStructure.USART_BaudRate = rate;  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	USART_InitStructure.USART_Parity = USART_Parity_No;  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;  
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;  
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;  
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;  
	USART_ClockInit(USART1, &USART_ClockInitStructure);  
	USART_Init(USART1, &USART_InitStructure );   

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure); 

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	//	USART_ITConfig(USART1, USART_IT_PE, ENABLE);
	//	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);

	USART_Cmd(USART1, ENABLE);  
}  
  
void tb_usart2_init(u32 rate) 
{  
	GPIO_InitTypeDef GPIO_InitStructure;  
	USART_InitTypeDef USART_InitStructure;   
	NVIC_InitTypeDef NVIC_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  
	USART_DeInit(USART2);  
	/* Configure USART Tx as alternate function push-pull */  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	/* Configure USART Rx as input floating */  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	USART_InitStructure.USART_BaudRate = rate;  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	USART_InitStructure.USART_Parity = USART_Parity_No;  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
	USART_Init(USART2, &USART_InitStructure );   

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

	//	USART_ITConfig(USART2, USART_IT_PE, ENABLE);
	//	USART_ITConfig(USART2, USART_IT_ERR, ENABLE);

	USART_Cmd(USART2, ENABLE); 
} 

void tb_usart3_init(u32 rate) 
{  
	GPIO_InitTypeDef GPIO_InitStructure;  
	USART_InitTypeDef USART_InitStructure;   
	NVIC_InitTypeDef NVIC_InitStructure; 


	/* config USART3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* USART3 GPIO config */
	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    

	/* Configure USART3 Rx (PB.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* USART3 mode config */
	USART_InitStructure.USART_BaudRate = rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

	USART_HalfDuplexCmd(USART3,ENABLE);

	USART_Cmd(USART3, ENABLE); 
} 

//从串口1发送一个字节
void tb_usart1_send_byte(u8 Data)
{
	USART_SendData(USART1, Data);
	return;
}

void tb_usart1_send_nbyte(u8 *Data, u16 size)
{
	u16 i = 0;
	for(i=0; i<size; i++) 
	{
		USART_SendData(USART1, Data[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
	return;
}

void tb_usart1_send_str(u8 *Data)
{
	while(*Data) 
	{
		USART_SendData(USART1, *Data++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
	return;
}

//从串口2发送一个字节
void tb_usart2_send_byte(u8 Data) 
{
	USART_SendData(USART2, Data);
	return;
}

void tb_usart2_send_nbyte(u8 *Data, u16 size) 
{
	u16 i = 0;
	for(i=0; i<size; i++)
	{
		USART_SendData(USART2, Data[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
	}
	return;
}

void tb_usart2_send_str(u8 *Data)
{
	while(*Data) 
	{
		USART_SendData(USART2, *Data++);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
	}
	return;
}

//从串口3发送一个字节
void tb_usart3_send_byte(u8 Data)
{
	USART_SendData(USART3, Data);
	return;
}

void tb_usart3_send_nbyte(u8 *Data, u16 size) 
{
	u16 i = 0;
	for(i=0; i<size; i++) 
	{
		USART_SendData(USART3, Data[i]);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
	}
	return;
}

void tb_usart3_send_str(u8 *Data) 
{
	while(*Data) 
	{
		USART_SendData(USART3, *Data++);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
	}
	return;
}


/**========================中断处理函数=============================**/
//串口收发中断处理函数
int _USART1_IRQHandler(void) 
{
	u8 sbuf_bak;
	static u16 buf_index = 0;

	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET) 
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);		
		sbuf_bak = USART_ReceiveData(USART1); 
		if(sbuf_bak == '<') 
		{
			uart1_mode = 4;
			buf_index = 0;
		}
		else if(uart1_mode == 0)
		{
			if(sbuf_bak == '$') 
			{
				uart1_mode = 1;
			} 
			else if(sbuf_bak == '#') 
			{
				uart1_mode = 2;
			} 
			else if(sbuf_bak == '{') 
			{
				uart1_mode = 3;
			}
			else if(sbuf_bak == '<') 
			{
				uart1_mode = 4;
			} 
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;

		if((uart1_mode == 4) && (sbuf_bak == '>'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		} 
		else if((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}    

		if(buf_index >= UART_BUF_SIZE)
		{
			buf_index = 0;
			uart1_mode = 0;
		}
		
	}

	//发送中断 用前在初始化的时候请打开
	//if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {   
	//USART_SendData(USARTy, TxBuffer1[TxCounter1++]);
	//}   
	return 0;
}

int _USART2_IRQHandler(void) 
{ 
	
	u8 sbuf_bak;
	static u8 led = 0;
	static u16 buf_index = 0;
	
	if(USART_GetFlagStatus(USART2,USART_IT_RXNE)==SET) 
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
	
	sbuf_bak = USART_ReceiveData(USART2); 
	//USART_SendData(USART1, sbuf_bak);
	
	if(sbuf_bak == '<') 
	{
		uart1_mode = 4;
		buf_index = 0;
	}
	else if(uart1_mode == 0)
	{
		if(sbuf_bak == '$') 
		{
			uart1_mode = 1;
		} 
		else if(sbuf_bak == '#') 
		{
			uart1_mode = 2;
		} 
		else if(sbuf_bak == '{') 
		{
			uart1_mode = 3;
		}
		else if(sbuf_bak == '<') 
		{
			uart1_mode = 4;
		} 
		buf_index = 0;
	}

	uart_receive_buf[buf_index++] = sbuf_bak;

	if((uart1_mode == 4) && (sbuf_bak == '>'))
	{
		uart2_close();
		uart_receive_buf[buf_index] = '\0';
		uart1_get_ok = 1;
	} 
	else if((uart1_mode == 1) && (sbuf_bak == '!'))
	{
		uart2_close();
		uart_receive_buf[buf_index] = '\0';
		uart1_get_ok = 1;
	}
	else if((uart1_mode == 2) && (sbuf_bak == '!'))
	{
		uart2_close();
		uart_receive_buf[buf_index] = '\0';
		uart1_get_ok = 1;
	}
	else if((uart1_mode == 3) && (sbuf_bak == '}'))
	{
		uart2_close();
		uart_receive_buf[buf_index] = '\0';
		uart1_get_ok = 1;
	}    

	if(buf_index >= UART_BUF_SIZE)
	{
		buf_index = 0;
		uart1_mode = 0;
	}
	
	
	

	if(led)
	{
		LED_OFF;
		led = ~ led;
	}
	else
	{
		LED_ON;
		led = ~ led;
	}


	
	
	//发送中断 用前在初始化的时候请打开
	//if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {   
	//	USART_SendData(USARTy, TxBuffer1[TxCounter1++]);
	//}  
	return 0;
}

int _USART3_IRQHandler(void)
{ 
	u8 sbuf_bak;
	static u16 buf_index = 0;
	if(USART_GetFlagStatus(USART3,USART_IT_RXNE)==SET) 
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);	
		sbuf_bak = USART_ReceiveData(USART3);
		if(uart1_get_ok)return 0;
		if(sbuf_bak == '<') 
		{
			uart1_mode = 4;
			buf_index = 0;
		}
		else if(uart1_mode == 0) 
		{
			if(sbuf_bak == '$') 
			{
				uart1_mode = 1;
			} 
			else if(sbuf_bak == '#') 
			{
				uart1_mode = 2;
			}
			else if(sbuf_bak == '{') 
			{
				uart1_mode = 3;
			}
			else if(sbuf_bak == '<') 
			{
				uart1_mode = 4;
			} 
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;

		if((uart1_mode == 4) && (sbuf_bak == '>'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart1_close();
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}    

		if(buf_index >= UART_BUF_SIZE)buf_index = 0;
	}

	//发送中断 用前在初始化的时候请打开
	//if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {   
	//	USART_SendData(USARTy, TxBuffer1[TxCounter1++]);
	//}  
	return 0;
}



void uart1_send_str(u8 *str) 
{
	tb_usart1_send_str(str);
}

void uart1_send_nbyte(u8 *Data, u16 size)
{
	tb_usart1_send_nbyte(Data, size);
}

void uart1_send_byte(u8 data)
{
	tb_usart1_send_byte(data);
}



void uart2_send_str(u8 *str)
{
	tb_usart2_send_str(str);	
}


void uart2_send_nbyte(u8 *Data, u16 size)
{
	tb_usart2_send_nbyte(Data, size);
}

void uart2_send_byte(u8 data)
{
	tb_usart2_send_byte(data);
}


void zx_uart_send_str(u8 *str)
{
	uart1_get_ok  = 1;
	tb_usart3_send_str(str);	
	uart1_get_ok = 0;
}


void uart3_send_str(u8 *str)
{
	tb_usart3_send_str(str);	
}


void uart3_send_nbyte(u8 *Data, u16 size)
{
	tb_usart3_send_nbyte(Data, size);
}

void uart3_send_byte(u8 data)
{
	tb_usart3_send_byte(data);
}



void handle_uart(void) 
{
	if(uart1_get_ok) 
	{
		//uart1_send_str(uart_receive_buf);
		if(uart1_mode == 1) 
		{					//命令模式
			//uart1_send_str((u8*) "cmd:");
			//uart1_send_str(uart_receive_buf);
            //uart1_send_str((u8*) "\n");
			parse_cmd(uart_receive_buf);			
		}
		else if(uart1_mode == 2) 
		{			//单个舵机调试
			//uart1_send_str("sig:");
			//uart1_send_str(uart_receive_buf);
			do_action(uart_receive_buf);
		}
		else if(uart1_mode == 3)
		{		//多路舵机调试
			//uart1_send_str("group:");
			//uart1_send_str(uart_receive_buf);
			do_action(uart_receive_buf);
		} 
		else if(uart1_mode == 4) 
		{		//存储模式
			//uart1_send_str("save:");
			//uart1_send_str(uart_receive_buf);
			action_save(uart_receive_buf);
		} 
		uart1_mode = 0;
		uart1_get_ok = 0;
		uart1_open();
		uart2_open();
	}
//	else
//	{
//		uart1_send_byte(uart1_get_ok);
//	}

	return;
}






