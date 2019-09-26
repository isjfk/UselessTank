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
    uint32_t header;
    size_t dataByteLength;
    uint32_t data[9];
    uint32_t crc;
} TankMsgStruct;

TankMsgStruct tankMsg;
uint8_t tankMsgBcd[sizeof(tankMsg) * 2 + 2];

CommonDataBufError tankMsgSend(float gyro[3], float accel[3], float compass[3]);

void tankMsgInit(void) {
    dataBufInit(&tankMsgBuf, tankMsgData, sizeof(tankMsgData));

    tankMsg.header = 0xAA55AA55;        // Actually [ 0x55, 0xAA, 0x55, 0xAA ] in message byte stream.
    tankMsg.dataByteLength = sizeof(tankMsg.data);
    for (size_t i = 0; i < arrayLen(tankMsg.data); i++) {
        tankMsg.data[i] = 0;
    }
    tankMsg.crc = 0;

    for (size_t i = 0; i < sizeof(tankMsgBcd); i++) {
        tankMsgBcd[i] = 0;
    }
    tankMsgBcd[sizeof(tankMsgBcd) - 2] = '\r';
    tankMsgBcd[sizeof(tankMsgBcd) - 1] = '\n';

    devCrcInit();
}

void tankMsgLoop(void) {
    float gyro[3];
    float accel[3];
    float compass[3];

    devMpu9250GetGyroFloat(gyro, NULL, NULL);
    devMpu9250GetAccelFloat(accel, NULL, NULL);
    devMpu9250GetCompassFloat(compass, NULL, NULL);

    tankMsgSend(gyro, accel, compass);
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

CommonDataBufError tankMsgSend(float gyro[3], float accel[3], float compass[3]) {
    size_t dataIndex = 0;

    tankMsg.data[dataIndex++] = (gyro != NULL) ? gyro[0] : 0;
    tankMsg.data[dataIndex++] = (gyro != NULL) ? gyro[1] : 0;
    tankMsg.data[dataIndex++] = (gyro != NULL) ? gyro[2] : 0;
    tankMsg.data[dataIndex++] = (accel != NULL) ? accel[0] : 0;
    tankMsg.data[dataIndex++] = (accel != NULL) ? accel[1] : 0;
    tankMsg.data[dataIndex++] = (accel != NULL) ? accel[2] : 0;
    tankMsg.data[dataIndex++] = (compass != NULL) ? compass[0] : 0;
    tankMsg.data[dataIndex++] = (compass != NULL) ? compass[1] : 0;
    tankMsg.data[dataIndex++] = (compass != NULL) ? compass[2] : 0;
    tankMsg.crc = devCrcByteArray((uint8_t *) tankMsg.data, tankMsg.dataByteLength);

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
