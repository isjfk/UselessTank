

#ifndef	_BLUETOOTH_H_
#define	_BLUETOOTH_H_

/*��������ģ���Դ*/
#define		BLUETOOTH_POWER_ON					(GPIO_SetBits(GPIOB, GPIO_Pin_0))
/*�ر�����ģ���Դ*/
#define		BLUETOOTH_POWER_OFF				(GPIO_ResetBits(GPIOB, GPIO_Pin_0))

/*�ø�BLT_CONF*/
#define		BLTCONTROL_BLT_CONF_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_5))
/*�õ�BLT_CONF*/
#define		BLTCONTROL_BLT_CONF_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_5))

void	Bluetooth_Init(void);										//���������ʼ��

#endif   // _BLUETOOTH_H_
