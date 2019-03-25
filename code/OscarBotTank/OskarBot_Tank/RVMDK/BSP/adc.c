#include "adc.h"

float bat_volt = 0;


/**
  * @brief  ��ʼ�����ڲ�����ص�ѹ��ADCͨ��
  *         ����Ŀ��ʹ��ADC1�ĵ�14��ͨ�����е�ص�ѹ��⣬��Ӧ��Ƭ���ĵ�PC4����ܽ�
  * @note   None
  * @param  None
  * @retval None
  */
void Adc_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// ʹ��GPIOC��ADC1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_ADC1, ENABLE );
	
	// ����ADC��Ƶϵ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	// ���õ�ѹ��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// ����ADC����
	ADC_DeInit(ADC1);                                   // ���ADC1����
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	// ����ADCΪ����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      // ����ADCΪ��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	// ����ADCΪ����ת��ģʽ
	
	// ����ADC�ⲿ����ģʽ������Ŀ����������Ʋ���
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	  // ����ADC���ݶ��뷽ʽΪ�Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 14;	                // ����Ŀ��ʹ�õ���ADC1��14ͨ��
	ADC_Init(ADC1, &ADC_InitStructure);                       // �����ϲ������õ�ADC1�ļĴ���
	ADC_Cmd(ADC1, ENABLE);                                    // ʹ��ADC1
	ADC_ResetCalibration(ADC1);                               // ��λУ׼
	while(ADC_GetResetCalibrationStatus(ADC1));               // �ȴ���λ���
	ADC_StartCalibration(ADC1);                               // ��ʼУ׼
	while(ADC_GetCalibrationStatus(ADC1));                    // �ȴ�У׼���
}

void Detect_Volt(void)
{
	float temp_volt = 0;
	ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5 );     // ����ADC��������		     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // �������ADCת��
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // �ȴ�ת�����
	temp_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // ����Ӳ�������ѹ�����Ͳο���ѹ���������ѹ
	bat_volt = bat_volt * 0.9 + temp_volt * 0.1;
}

void Init_Volt(void)
{
	float temp_volt = 0;
	int i = 0;
	
	ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5 );     // ����ADC��������		     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // �������ADCת��
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // �ȴ�ת�����
	bat_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // ����Ӳ�������ѹ�����Ͳο���ѹ���������ѹ

	for(int i = 0; i < 10; i++)
	{
		ADC_RegularChannelConfig(ADC1, 14, 1, ADC_SampleTime_239Cycles5 );     // ����ADC��������		     
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // �������ADCת��
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // �ȴ�ת�����
		temp_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // ����Ӳ�������ѹ�����Ͳο���ѹ���������ѹ
		bat_volt = bat_volt * 0.9 + temp_volt * 0.1;
	}
	
}



///**
//  * @brief  ��ʼ�����ڲ���CCD
//  *         ����Ŀ��ʹ��ADC1�ĵ�15��ͨ�����м�⣬��Ӧ��Ƭ���ĵ�PC5����ܽ�
//  * @note   None
//  * @param  None
//  * @retval None
//  */
//void CCD_Init(void)
//{    
// 	ADC_InitTypeDef ADC_InitStructure; 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	// ʹ��GPIOC��ADC1ʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_ADC1, ENABLE );
//	
//	// ����ADC��Ƶϵ��
//	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
//	
//	// ���õ�ѹ��������
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	// ����ADC����
//	ADC_DeInit(ADC1);                                   // ���ADC1����
//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	// ����ADCΪ����ģʽ
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      // ����ADCΪ��ͨ��ģʽ
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	// ����ADCΪ����ת��ģʽ
//	
//	// ����ADC�ⲿ����ģʽ������Ŀ����������Ʋ���
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//	
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	  // ����ADC���ݶ��뷽ʽΪ�Ҷ���
//	ADC_InitStructure.ADC_NbrOfChannel = 15;	                // ����Ŀ��ʹ�õ���ADC1��14ͨ��
//	ADC_Init(ADC1, &ADC_InitStructure);                       // �����ϲ������õ�ADC1�ļĴ���
//	ADC_Cmd(ADC1, ENABLE);                                    // ʹ��ADC1
//	ADC_ResetCalibration(ADC1);                               // ��λУ׼
//	while(ADC_GetResetCalibrationStatus(ADC1));               // �ȴ���λ���
//	ADC_StartCalibration(ADC1);                               // ��ʼУ׼
//	while(ADC_GetCalibrationStatus(ADC1));                    // �ȴ�У׼���
//}

//void Detect_CCD(void)
//{
//	ADC_RegularChannelConfig(ADC1, 15, 1, ADC_SampleTime_239Cycles5 );     // ����ADC��������		     
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		                             // �������ADCת��
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));                        // �ȴ�ת�����
//	bat_volt = ADC_GetConversionValue(ADC1) * 3.3 * 6 / 4096;              // ����Ӳ�������ѹ�����Ͳο���ѹ���������ѹ
//}


