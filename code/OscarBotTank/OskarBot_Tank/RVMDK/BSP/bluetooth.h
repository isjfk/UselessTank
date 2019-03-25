

#ifndef	_BLUETOOTH_H_
#define	_BLUETOOTH_H_

/*开启蓝牙模块电源*/
#define		BLUETOOTH_POWER_ON					(GPIO_SetBits(GPIOB, GPIO_Pin_0))
/*关闭蓝牙模块电源*/
#define		BLUETOOTH_POWER_OFF				(GPIO_ResetBits(GPIOB, GPIO_Pin_0))

/*置高BLT_CONF*/
#define		BLTCONTROL_BLT_CONF_HIGH		(GPIO_SetBits(GPIOC, GPIO_Pin_5))
/*置低BLT_CONF*/
#define		BLTCONTROL_BLT_CONF_LOW			(GPIO_ResetBits(GPIOC, GPIO_Pin_5))

void	Bluetooth_Init(void);										//蓝牙外设初始化

#endif   // _BLUETOOTH_H_
