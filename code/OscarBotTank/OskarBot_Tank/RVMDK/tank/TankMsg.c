#include <stdio.h>
#include <string.h>
#include <math.h>

#include "TankMsg.h"
#include "Tank.h"
#include "common/CommonMisc.h"
#include "system/SysIrq.h"
#include "board/Board.h"
#include "device/DevCrc.h"
#include "device/DevMpu9250.h"
#include "device/DevMotor.h"
#include "device/DevUsart.h"

#define gyroDegree2Radian(d)        (d * M_PI / 180)
#define accelG2Ms2(g)               (g * 9.80665)
#define microTesla2Tesla(mt)        (mt / 1000000.0)

uint8_t tankMsgData[512];
CommonDataBuf tankMsgBuf;

typedef struct {
    float gyro[3];
    float accel[3];
    float compass[3];
    float quat[4];
    uint32_t motorEncoderLeft;
    uint32_t motorEncoderRight;
} TankMsgData;

typedef struct {
    uint32_t startTag;
    uint32_t header;
    uint32_t timestamp;
    uint32_t dataLength;
    TankMsgData data;
    uint32_t crc;
} TankMsg;

TankMsg tankMsg;

CommonDataBufError tankMsgSend(void);

void tankMsgInit(void) {
    dataBufInit(&tankMsgBuf, tankMsgData, sizeof(tankMsgData));

    memset(&tankMsg, 0, sizeof(tankMsg));
    tankMsg.startTag = 0xAA55AA55;          // Actually [ 0x55, 0xAA, 0x55, 0xAA ] in message byte stream.
    tankMsg.header = 0x00000001;            // Protocol version 1. [ 0x01, 0x00, 0x00, 0x00 ] in message byte stream.
    tankMsg.dataLength = sizeof(tankMsg.data);
}

void tankMsgLoop(void) {
    int8_t accuracy;
    inv_time_t timestamp;

    devMpu9250GetGyroFloat(tankMsg.data.gyro, &accuracy, &timestamp);
    // Use gyro timestamp as TankMsg timestamp.
    tankMsg.timestamp = timestamp;
    devMpu9250GetAccelFloat(tankMsg.data.accel, &accuracy, &timestamp);
    devMpu9250GetCompassFloat(tankMsg.data.compass, &accuracy, &timestamp);
    devMpu9250GetQuatFloat(tankMsg.data.quat, &accuracy, &timestamp);

    tankMsg.data.gyro[0] = gyroDegree2Radian(tankMsg.data.gyro[0]);
    tankMsg.data.gyro[1] = gyroDegree2Radian(tankMsg.data.gyro[1]);
    tankMsg.data.gyro[2] = gyroDegree2Radian(tankMsg.data.gyro[2]);
    tankMsg.data.accel[0] = accelG2Ms2(tankMsg.data.accel[0]);
    tankMsg.data.accel[1] = accelG2Ms2(tankMsg.data.accel[1]);
    tankMsg.data.accel[2] = accelG2Ms2(tankMsg.data.accel[2]);
    tankMsg.data.compass[0] = microTesla2Tesla(tankMsg.data.compass[0]);
    tankMsg.data.compass[1] = microTesla2Tesla(tankMsg.data.compass[1]);
    tankMsg.data.compass[2] = microTesla2Tesla(tankMsg.data.compass[2]);

    tankMsg.data.motorEncoderLeft = devMotorGetLeftEncoder();
    tankMsg.data.motorEncoderRight = devMotorGetRightEncoder();

    tankMsgSend();
}

CommonDataBufError tankMsgReadByte(uint8_t *data) {
    return dataBufReadByte(&tankMsgBuf, data);
}

CommonDataBufError tankMsgSend(void) {
    devCrcReset();
    tankMsg.crc = devStdCrc32ByteArray(&tankMsg.header, (sizeof(tankMsg) - sizeof(tankMsg.startTag) - sizeof(tankMsg.crc)));

    CommonDataBufError bufStatus = dataBufAppendByteArray(&tankMsgBuf, (uint8_t *) &tankMsg, sizeof(tankMsg));
    if (bufStatus == COMMON_DATABUF_OK) {
        irqLock();
        if (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET) {
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        }
        irqUnLock();
    }

    return bufStatus;
}
