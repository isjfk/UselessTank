#include "adc.h"

float volatile bat_volt = 0;


/**
  * @brief  初始化用于测量电池电压的ADC通道
  *         本项目中使用ADC1的第14个通道进行电池电压检测，对应单片机的的PC4这个管脚
  * @note   None
  * @param  None
  * @retval None
  */
void Adc_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 使能GPIOC和ADC1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_ADC1, ENABLE );
	
	// 设置ADC分频系数
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	// 配置电压测量引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// 配置ADC参数
	ADC_DeInit(ADC1);                                   // 清除ADC1设置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	// 设置ADC为独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      // 设置ADC为单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	// 设置ADC为单次转换模式
	
	// 禁用ADC外部触发模式，本项目中由软件控制测量
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	  // 设置ADC数据对齐方式为右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 14;	                // 本项目中使用的是ADC1的14通道
	ADC_Init(ADC1, &ADC_InitStructure);                       // 将以上参数配置到ADC1的寄存器
	ADC_Cmd(ADC1, ENABLE);                                    // 使能ADC1
	ADC_ResetCalibration(ADC1);                               // 复位校准
	while(ADC_GetResetCalibrationStatus(ADC1));               // 等待复位完成
	ADC_StartCalibration(ADC1);                               // 开始校准
	while(ADC_GetCalibrationStatus(ADC1));                    // 等待校准完成
}

void Detect_Volt(void)
{
	float temp_volt = 0;
	ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5 );     // 设置ADC采样周期		     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // 软件启动ADC转换
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // 等待转换完成
	temp_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // 根据硬件电阻分压比例和参考电压，计算出电压
	bat_volt = bat_volt * 0.9 + temp_volt * 0.1;
}

void Init_Volt(void)
{
	float temp_volt = 0;
	int i = 0;
	
	ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5 );     // 设置ADC采样周期		     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // 软件启动ADC转换
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // 等待转换完成
	bat_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // 根据硬件电阻分压比例和参考电压，计算出电压

	for(int i = 0; i < 10; i++)
	{
		ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5 );     // 设置ADC采样周期		     
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // 软件启动ADC转换
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // 等待转换完成
		temp_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // 根据硬件电阻分压比例和参考电压，计算出电压
		bat_volt = bat_volt * 0.9 + temp_volt * 0.1;
	}
	
}



///**
//  * @brief  初始化用于测量CCD
//  *         本项目中使用ADC1的第15个通道进行检测，对应单片机的的PC5这个管脚
//  * @note   None
//  * @param  None
//  * @retval None
//  */
//void CCD_Init(void)
//{    
// 	ADC_InitTypeDef ADC_InitStructure; 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	// 使能GPIOC和ADC1时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_ADC1, ENABLE );
//	
//	// 设置ADC分频系数
//	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
//	
//	// 配置电压测量引脚
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	// 配置ADC参数
//	ADC_DeInit(ADC1);                                   // 清除ADC1设置
//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	// 设置ADC为独立模式
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      // 设置ADC为单通道模式
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	// 设置ADC为单次转换模式
//	
//	// 禁用ADC外部触发模式，本项目中由软件控制测量
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//	
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	  // 设置ADC数据对齐方式为右对齐
//	ADC_InitStructure.ADC_NbrOfChannel = 15;	                // 本项目中使用的是ADC1的14通道
//	ADC_Init(ADC1, &ADC_InitStructure);                       // 将以上参数配置到ADC1的寄存器
//	ADC_Cmd(ADC1, ENABLE);                                    // 使能ADC1
//	ADC_ResetCalibration(ADC1);                               // 复位校准
//	while(ADC_GetResetCalibrationStatus(ADC1));               // 等待复位完成
//	ADC_StartCalibration(ADC1);                               // 开始校准
//	while(ADC_GetCalibrationStatus(ADC1));                    // 等待校准完成
//}

//void Detect_CCD(void)
//{
//	ADC_RegularChannelConfig(ADC1, 15, 1, ADC_SampleTime_239Cycles5 );     // 设置ADC采样周期		     
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // 软件启动ADC转换
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // 等待转换完成
//	bat_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // 根据硬件电阻分压比例和参考电压，计算出电压
//}


