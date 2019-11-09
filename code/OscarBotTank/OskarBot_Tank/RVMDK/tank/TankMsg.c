#include <stdio.h>
#include <string.h>

#include "TankMsg.h"
#include "Tank.h"
#include "common/CommonMisc.h"
#include "system/SysIrq.h"
#include "board/Board.h"
#include "device/DevCrc.h"
#include "device/DevMpu9250.h"
#include "device/DevUsart.h"

uint8_t tankMsgTypeBinary = 1;      // 0: command output bcd string; 1: command output byte array;

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
uint8_t tankMsgBcd[sizeof(tankMsg) * 2 + 2];

CommonDataBufError tankMsgSend(void);

void tankMsgInit(void) {
    dataBufInit(&tankMsgBuf, tankMsgData, sizeof(tankMsgData));

    memset(&tankMsg, 0, sizeof(tankMsg));
    tankMsg.startTag = 0xAA55AA55;          // Actually [ 0x55, 0xAA, 0x55, 0xAA ] in message byte stream.
    tankMsg.header = 0x00000001;            // Protocol version 1. [ 0x01, 0x00, 0x00, 0x00 ] in message byte stream.
    tankMsg.dataLength = sizeof(tankMsg.data);

    memset(&tankMsgBcd, 0, sizeof(tankMsgBcd));
    tankMsgBcd[sizeof(tankMsgBcd) - 2] = '\r';
    tankMsgBcd[sizeof(tankMsgBcd) - 1] = '\n';
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

    tankMsg.data.motorEncoderLeft = boardEncoderLeftGet();
    tankMsg.data.motorEncoderRight = boardEncoderRightGet();

    tankMsgSend();
}

CommonDataBufError tankMsgReadByte(uint8_t *data) {
    return dataBufReadByte(&tankMsgBuf, data);
}

uint16_t byte2bcd(uint8_t b) {
    static const uint8_t byte2bcdArray[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

    uint16_t bcd = 0;
    bcd |= byte2bcdArray[(b >> 4) & 0xF];
    bcd |= byte2bcdArray[b & 0xF] << 8;

    return bcd;
}

CommonDataBufError tankMsgSend(void) {
    devCrcReset();
    tankMsg.crc = devStdCrc32ByteArray(&tankMsg.header, (sizeof(tankMsg) - sizeof(tankMsg.startTag) - sizeof(tankMsg.crc)));

    CommonDataBufError bufStatus;
    if (tankMsgTypeBinary) {
        bufStatus = dataBufAppendByteArray(&tankMsgBuf, (uint8_t *) &tankMsg, sizeof(tankMsg));
    } else {
        uint8_t *byteArray = (uint8_t *) &tankMsg;
        uint16_t *bcdTemplate = (uint16_t *) tankMsgBcd;
        for (size_t i = 0; i < sizeof(tankMsg); i++) {
            bcdTemplate[i] = byte2bcd(byteArray[i]);
        }

        bufStatus = dataBufAppendByteArray(&tankMsgBuf, tankMsgBcd, sizeof(tankMsgBcd));
    }

    if (bufStatus == COMMON_DATABUF_OK) {
        irqLock();
        if (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET) {
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        }
        irqUnLock();
    }

    return bufStatus;
}
