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
	// ϵͳʱ�ӳ�ʼ�� System clock initialization
	SystemInit();
	
	// ��ص�ѹ����ʼ��
	Adc_Init();
	
	// PS2�ֱ���ʼ�� Handle initialization
	PSX_init();	
	
	// �ŷ������裨���ֶ������ʼ�� Servo peripherals initialization
	servo_init();
	
	// ���ڳ�ʼ����������115200 Serial port initialization, baud rate of 115200
	tb_usart1_init(9600);
	tb_usart2_init(115200);
	tb_usart3_init(115200);
	
	
	// �ж�ʹ�� Enable interruption
	tb_interrupt_open();
	
	// ���߶����� Bus servo output
	zx_uart_send_str((u8 *)"#255P1500T2000!");
	
	// �����жϳ�ʼ�� Millisecond interrupt initialization
	SysTick_Int_Init();
	
	// SPI Flash�洢����ʼ�� Flash memory Initialization
//	W25Q_Init();	
//	if(W25Q_TYPE != W25Q64)
//	{
//		while(1)BEEP_ON;
//	}

	// ����������ʼ�� Left encoder initialization
	Left_Encoder_Init();
	
	// �Ҳ��������ʼ�� Right encoder initialization
	Right_Encoder_Init();
	
	// ���ٵ��PWM��ʼ�� Motor PWM Initialization
	Motor_Init(7199, 0);
	
	// ��������LED��ʼ�� Buzzer and LED initialization
	Beep_Led_Init();
	
	// MPU9250 DMP ��ʼ�� MPU9250 DMP initialization
//	while(mpu_dmp_init())
//	{   
// 		delay_ms(200);
//	}
	
	Sys_OK_Sound();
	
	
	joy_left_pwm = 0;
	joy_right_pwm = 0;
	
	// ��ʼ����ص�ѹ
	Init_Volt();
	
  /* ��ѭ�� Infinite loop */
  while (1)
  {
		// ��ص�ѹ���
		Detect_Volt();
		
		// ���˲ο�ϵͳ�㷨 (AHRS)Attitude and heading reference system
		AHRS();
		
		// ps2�ֱ������ PS2 handle command processing
		handle_ps2();
		
		// ps2������Ӧ Key response
		handle_button();
		
		// ������Ϣ���� Serial message processing
		handle_uart();
		
		// ���ִ�ж��� Servos perform action
		handle_action();
		
		// ����1�����ص�ѹ
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
