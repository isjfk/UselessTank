#ifndef __W25Q64_H__
#define __W25Q64_H__

#include "stm32f10x.h"
#include "sys.h"

#define TRUE 1
#define FALSE 0

//W25X??/Q??????	   
//W25Q80 ID  0XEF13
//W25Q16 ID  0XEF14
//W25Q32 ID  0XEF15
//W25Q32 ID  0XEF16	

#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16

#define W25Q64_SECTOR_SIZE	0x1000		//4K
#define W25Q64_SECTOR_NUM	2048		//8*1024/4 = 2048


extern u16 W25Q_TYPE;//???????flash????

#define	W25Q_CS PAout(4)  //??FLASH	
				 
////////////////////////////////////////////////////////////////////////////
 
//???
#define W25X_WriteEnable			0x06 		//???
#define W25X_WriteDisable			0x04 		//???
#define W25X_ReadStatusReg  	0x05 		//??????
#define W25X_WriteStatusReg		0x01 		//??????
#define W25X_ReadData					0x03 		//???
#define W25X_FastReadData			0x0B 		//
#define W25X_FastReadDual			0x3B 
#define W25X_PageProgram			0x02 
#define W25X_BlockErase				0xD8 
#define W25X_SectorErase			0x20 
#define W25X_ChipErase				0xC7 		
#define W25X_PowerDown				0xB9 		//??
#define W25X_ReleasePowerDown	0xAB 		//??
#define W25X_DeviceID					0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

void W25Q_Init(void);
void W25Q_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //??flash
void W25Q_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//??flash


u16  W25Q_ReadID(void);  	    //??FLASH ID
u8	 W25Q_ReadSR(void);        //??????? 
void W25Q_Write_SR(u8 sr);  	//??????
void W25Q_Write_Enable(void);  //??? 
void W25Q_Write_Disable(void);	//???
void W25Q_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void W25Q_Erase_Chip(void);    	  //????
void W25Q_Erase_Sector(u32 Dst_Addr);//????
void W25Q_Wait_Busy(void);           //????
void W25Q_PowerDown(void);           //??????
void W25Q_WAKEUP(void);			  //??


void SPI1_Init(void);			 //???SPI?
void SPI1_SetSpeed(u8 SpeedSet); //??SPI??   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI????????

#define w25x_init() W25Q_Init()
#define w25x_readId()	W25Q_ReadID()
#define w25x_read(buf, addr, len) W25Q_Read(buf, addr, len)
#define w25x_write(buf, addr, len) W25Q_Write(buf, addr, len)
#define w25x_erase_sector(addr) W25Q_Erase_Sector(addr)

#endif
