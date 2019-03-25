/**
  ******************************************************************************
  * @file    
  * @author: Oskar Wei
	* @Mail: 990092230@qq.com
  * @version 1.1
  * @date    
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT Oskarbot 2017
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "led.h"
#include "usart.h"
#include "delay.h"
#include "ahrs.h"
#include "stdbool.h"
#include "motor.h"
#include "encoder.h"
#include "ps2.h"
#include "servo.h"
#include "main.h"
#include "string.h"
#include "w25q64.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "mpu9250.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "adc.h"




/* Private define ------------------------------------------------------------*/
/* Extern define ------------------------------------------------------------*/
extern u8 psx_buf[];
extern int Moto1,Moto2;  
extern int joy_left_pwm, joy_right_pwm;
extern u8 uart_receive_buf[UART_BUF_SIZE], uart1_get_ok, uart1_mode;
extern float bat_volt;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



int main(void)
{
	// 系统时钟初始化 System clock initialization
	SystemInit();
	
	// 电池电压检测初始化
	Adc_Init();
	
	// PS2手柄初始化 Handle initialization
	PSX_init();	
	
	// 伺服类外设（数字舵机）初始化 Servo peripherals initialization
	servo_init();
	
	// 串口初始化，波特率115200 Serial port initialization, baud rate of 115200
	tb_usart1_init(9600);
	tb_usart2_init(115200);
	tb_usart3_init(115200);
	
	
	// 中断使能 Enable interruption
	tb_interrupt_open();
	
	// 总线舵机输出 Bus servo output
	zx_uart_send_str((u8 *)"#255P1500T2000!");
	
	// 毫秒中断初始化 Millisecond interrupt initialization
	SysTick_Int_Init();
	
	// SPI Flash存储器初始化 Flash memory Initialization
//	W25Q_Init();	
//	if(W25Q_TYPE != W25Q64)
//	{
//		while(1)BEEP_ON;
//	}

	// 左侧编码器初始化 Left encoder initialization
	Left_Encoder_Init();
	
	// 右侧编码器初始化 Right encoder initialization
	Right_Encoder_Init();
	
	// 减速电机PWM初始化 Motor PWM Initialization
	Motor_Init(7199, 0);
	
	// 蜂鸣器和LED初始化 Buzzer and LED initialization
	Beep_Led_Init();
	
	// MPU9250 DMP 初始化 MPU9250 DMP initialization
//	while(mpu_dmp_init())
//	{   
// 		delay_ms(200);
//	}
	
	Sys_OK_Sound();
	
	
	joy_left_pwm = 0;
	joy_right_pwm = 0;
	
	// 初始化电池电压
	Init_Volt();
	
  /* 主循环 Infinite loop */
  while (1)
  {
		// 电池电压检测
		Detect_Volt();
		
		// 航姿参考系统算法 (AHRS)Attitude and heading reference system
		AHRS();
		
		// ps2手柄命令处理 PS2 handle command processing
		handle_ps2();
		
		// ps2按键响应 Key response
		handle_button();
		
		// 串口消息处理 Serial message processing
		handle_uart();
		
		// 舵机执行动作 Servos perform action
		handle_action();
		
		// 串口1输出电池电压
		//printf("Vottage:%4.1f\r\n", bat_volt);
		//printf("[0]:%d, [1]:%d, [2]:%d, [3]:%d, [4]:%d, [5]:%d, [6]:%d, [7]:%d, [8]:%d, M1:%d ~\r\n", psx_buf[0], psx_buf[1], psx_buf[2], psx_buf[3], psx_buf[4], psx_buf[5], psx_buf[6], psx_buf[7], psx_buf[8], myabs(Moto1));
  }
}










#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
