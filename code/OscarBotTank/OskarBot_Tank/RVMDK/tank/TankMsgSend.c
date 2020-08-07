#include <stdio.h>
#include <string.h>
#include <math.h>

#include "TankMsgSend.h"
#include "Tank.h"
#include "TankMsg.h"
#include "TankMsgRecv.h"
#include "common/CommonMisc.h"
#include "system/SysIrq.h"
#include "system/SysTime.h"
#include "board/Board.h"
#include "device/DevCrc.h"
#include "device/DevMpu9250.h"
#include "device/DevMotor.h"
#include "device/DevUsart.h"

#define gyroDegree2Radian(d)        (d * M_PI / 180)
#define accelG2Ms2(g)               (g * 9.80665)
#define microTesla2Tesla(mt)        (mt / 1000000.0)

uint32_t tankMsgSendSuccessMsgCount = 0;
uint32_t tankMsgSendOverflowMsgCount = 0;

static uint8_t tankMsgSendData[sizeof(TankMsg) * 5];
static CommonDataBuf tankMsgSendBuf;

static TankMsgPacket tankMsgPacket;
static TankMsg *tankMsg = &(tankMsgPacket.tankMsg);

static SysTimeLoop sendTankStatusLoop;

CommonError tankMsgSend(void);
CommonError tankMsgSendSensorMsg(void);
CommonError tankMsgSendTankStatus(void);

void tankMsgSendInit(void) {
    dataBufInit(&tankMsgSendBuf, tankMsgSendData, sizeof(tankMsgSendData));
    tankMsgPacketInit(&tankMsgPacket);
    sysTimeLoopStart(&sendTankStatusLoop, 1000);
}

void tankMsgSendLoop(void) {
    tankMsgSendSensorMsg();
    tankMsgSendTankStatus();
}

CommonError tankMsgSendTextLog(uint32_t level, void *msg, size_t size) {    
    TankMsgTextLog *data = tankMsgReqInitByType(tankMsg, TankMsgTextLog);
    uint32_t dataHeaderSize = tankMsgDataSizeOfType(TankMsgTextLog);

    uint32_t sizeMax = tankMsgDataSizeMax(tankMsg) - dataHeaderSize;
    size = (size <= sizeMax) ? size : sizeMax;

    tankMsg->timestampMs = sysTimeCurrentMs();
    tankMsg->dataLength = dataHeaderSize + size;
    data->level = level;
    data->msgSize = size;
    memcpy(data->msg, msg, size);

    return tankMsgSend();
}

CommonError tankMsgSendBinLog(uint32_t level, void *msg, size_t size) {
    TankMsgBinLog *data = tankMsgReqInitByType(tankMsg, TankMsgBinLog);
    uint32_t dataHeaderSize = tankMsgDataSizeOfType(TankMsgBinLog);

    uint32_t sizeMax = tankMsgDataSizeMax(tankMsg) - dataHeaderSize;
    size = (size <= sizeMax) ? size : sizeMax;

    tankMsg->timestampMs = sysTimeCurrentMs();
    tankMsg->dataLength = dataHeaderSize + size;
    data->level = level;
    data->msgSize = size;
    memcpy(data->msg, msg, size);

    return tankMsgSend();
}

CommonError tankMsgSendSensorMsg(void) {
    int8_t accuracy;
    inv_time_t timestampMs;

    TankMsgSensorData *data = tankMsgReqInitByType(tankMsg, TankMsgSensorData);

    devMpu9250GetGyroFloat(data->gyro, &accuracy, &timestampMs);
    devMpu9250GetAccelFloat(data->accel, &accuracy, NULL);
    devMpu9250GetCompassFloat(data->compass, &accuracy, NULL);
    devMpu9250GetQuatFloat(data->quat, &accuracy, NULL);

    data->motorEncoderLeft = devMotorGetLeftEncoder();
    data->motorEncoderRight = devMotorGetRightEncoder();

    // Use gyro timestamp as TankMsg timestamp.
    tankMsg->timestampMs = timestampMs;

    // Convert unit to ROS standard
    data->gyro[0] = gyroDegree2Radian(data->gyro[0]);
    data->gyro[1] = gyroDegree2Radian(data->gyro[1]);
    data->gyro[2] = gyroDegree2Radian(data->gyro[2]);
    data->accel[0] = accelG2Ms2(data->accel[0]);
    data->accel[1] = accelG2Ms2(data->accel[1]);
    data->accel[2] = accelG2Ms2(data->accel[2]);
    data->compass[0] = microTesla2Tesla(data->compass[0]);
    data->compass[1] = microTesla2Tesla(data->compass[1]);
    data->compass[2] = microTesla2Tesla(data->compass[2]);

    return tankMsgSend();
}

CommonError tankMsgSendTankStatus(void) {
    if (!sysTimeLoopShouldEnter(&sendTankStatusLoop)) {
        return COMMON_ERROR_OK;
    }

    TankMsgTankStatus *data = tankMsgReqInitByType(tankMsg, TankMsgTankStatus);
    tankMsg->timestampMs = sysTimeCurrentMs();

    data->isShutdown = isShutdown();
    data->isEmergencyStop = pdbIsStopButtonDown();
    data->isBatteryVoltageLow = boardIsBatteryLow();
    data->isBatteryVoltageVeryLow = boardIsBatteryVeryLow();
    data->batteryVoltage = boardGetBatteryVoltage();
    data->tankMsgSendSuccessMsgCount = tankMsgSendSuccessMsgCount;
    data->tankMsgSendOverflowMsgCount = tankMsgSendOverflowMsgCount;
    data->tankMsgRecvValidMsgCount = tankMsgRecvValidMsgCount;
    data->tankMsgRecvIllegalMsgCount = tankMsgRecvIllegalMsgCount;
    data->tankMsgRecvUnsupportedMsgCount = tankMsgRecvUnsupportedMsgCount;
    data->tankMsgRecvInternalErrorCount = tankMsgRecvInternalErrorCount;

    return tankMsgSend();
}

CommonError tankMsgSend(void) {
    tankMsg->crcHeader = devStdCrc32ByteArray(tankMsgHeaderAddr(tankMsg), tankMsgHeaderCrcSize(tankMsg));
    tankMsg->crcHeaderData = devStdCrc32UpdateByteArray(tankMsgDataAddr(tankMsg), tankMsgDataCrcSize(tankMsg));

    CommonDataBufError bufStatus = dataBufAppendByteArray(&tankMsgSendBuf, tankMsgPacketAddr(&tankMsgPacket), tankMsgPacketSize(&tankMsgPacket));
    if (bufStatus == COMMON_ERROR_OK) {
        tankMsgSendSuccessMsgCount++;

        irqLock();
        if (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET) {
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        }
        irqUnLock();
    } else {
        tankMsgSendOverflowMsgCount++;
    }

    return bufStatus;
}

CommonError tankMsgSendBufReadByte(uint8_t *data) {
    return dataBufReadByte(&tankMsgSendBuf, data);
}
