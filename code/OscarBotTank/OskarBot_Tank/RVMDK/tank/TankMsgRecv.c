#include <stdio.h>
#include <string.h>
#include <math.h>

#include "TankMsgRecv.h"
#include "Tank.h"
#include "TankMsg.h"
#include "common/CommonMisc.h"
#include "system/SysIrq.h"
#include "system/SysTime.h"
#include "board/Board.h"
#include "device/DevCrc.h"
#include "device/DevMpu9250.h"
#include "device/DevMotor.h"
#include "device/DevUsart.h"

uint32_t tankMsgRecvSeqPrev = UINT32_MAX;
uint32_t tankMsgRecvSeqCurr = UINT32_MAX;
uint32_t tankMsgRecvValidMsgCount = 0;
uint32_t tankMsgRecvIllegalMsgCount = 0;
uint32_t tankMsgRecvUnsupportedMsgCount = 0;
uint32_t tankMsgRecvInternalErrorCount = 0;

static uint8_t tankMsgRecvData[sizeof(TankMsg) * 5];
static CommonDataBuf tankMsgRecvBuf;

static TankMsgPacket tankMsgPacket;
static TankMsg *tankMsg = &(tankMsgPacket.tankMsg);

#define STATE_START_TAG     1
#define STATE_MSG_HEADER    2
#define STATE_MSG_DATA      3

static uint8_t state = STATE_START_TAG;
static size_t readSize = 0;
static const size_t startTagSizeMin = 5;

static void tankMsgRecvOnReceived(TankMsg *tankMsg);
static inline bool tankMsgRecvIsHeaderValid(TankMsg *tankMsg);
static inline bool tankMsgRecvIsDataValid(TankMsg *tankMsg);

void tankMsgRecvInit(void) {
    dataBufInit(&tankMsgRecvBuf, tankMsgRecvData, sizeof(tankMsgRecvData));
    tankMsgPacketInit(&tankMsgPacket);
}

void tankMsgRecvLoop(void) {
    uint8_t data;
    while (dataBufReadByte(&tankMsgRecvBuf, &data) == COMMON_ERROR_OK) {
        switch (state) {
        case STATE_START_TAG:
            if ((data == 0xFF) && (readSize > startTagSizeMin)) {
                readSize = 0;
                state = STATE_MSG_HEADER;
            } else if (data == 0x55) {
                readSize++;
            } else {
                readSize = 0;
            }
            break;
        case STATE_MSG_HEADER:
            tankMsgSet(tankMsg, readSize++, data);
            if (readSize >= tankMsgHeaderSize(tankMsg)) {
                if (tankMsgRecvIsHeaderValid(tankMsg)) {
                    state = STATE_MSG_DATA;
                } else {
                    tankMsgRecvIllegalMsgCount++;
                    readSize = 0;
                    state = STATE_START_TAG;
                }
            }
            break;
        case STATE_MSG_DATA:
            tankMsgSet(tankMsg, readSize++, data);
            if (readSize >= tankMsgSize(tankMsg)) {
                if (tankMsgRecvIsDataValid(tankMsg)) {
                    tankMsgRecvValidMsgCount++;
                    tankMsgRecvOnReceived(tankMsg);
                } else {
                    tankMsgRecvIllegalMsgCount++;
                }
                readSize = 0;
                state = STATE_START_TAG;
            }
            break;
        default:
            tankMsgRecvInternalErrorCount++;
            readSize = 0;
            state = STATE_START_TAG;
            break;
        }
    }
}

static void tankMsgRecvOnTankMsgCtrlTank(TankMsg *tankMsg, TankMsgCtrlTank *msgData) {
    tankThrottleSet(msgData->x / 300.0);
    tankYawSet(msgData->yaw / 300.0);
}

static void tankMsgRecvOnTankMsgRosStatus(TankMsg *tankMsg, TankMsgRosStatus *msgData) {
    // Max timestamp difference in ms which a time correction should execute
    uint32_t sysTimeMsMaxDiff = 100;
    // Time offset in ms between ROS send the message and control board start to process the message
    uint32_t sysTimeMsOffset = 30;

    uint32_t rosSysTimeMs = tankMsg->timestampMs;
    if ((rosSysTimeMs - sysTimeCurrentMs()) > sysTimeMsMaxDiff) {
        sysTimeSetMs(rosSysTimeMs + sysTimeMsOffset);
    }
}

static void tankMsgRecvUpdateStatus(TankMsg *tankMsg) {
    tankMsgRecvSeqPrev = tankMsgRecvSeqCurr;
    tankMsgRecvSeqCurr = tankMsg->seq;

    if (tankMsgRecvSeqCurr < tankMsgRecvSeqPrev) {
        alarmRosOk();
    }
}

static void tankMsgRecvOnReceived(TankMsg *tankMsg) {
    tankMsgRecvUpdateStatus(tankMsg);

    switch (tankMsg->dataType) {
    case tankMsgDataType(TankMsgCtrlTank):
        tankMsgRecvOnTankMsgCtrlTank(tankMsg, tankMsgDataPtrOfType(tankMsg, TankMsgCtrlTank));
        break;
    case tankMsgDataType(TankMsgRosStatus):
        tankMsgRecvOnTankMsgRosStatus(tankMsg, tankMsgDataPtrOfType(tankMsg, TankMsgRosStatus));
        break;
    default:
        tankMsgRecvUnsupportedMsgCount++;
        break;
    }
}

static inline bool tankMsgRecvIsHeaderValid(TankMsg *tankMsg) {
    uint32_t crc32 = devStdCrc32ByteArray(tankMsgHeaderAddr(tankMsg), tankMsgHeaderCrcSize(tankMsg));
    return crc32 == tankMsg->crcHeader;
}

static inline bool tankMsgRecvIsDataValid(TankMsg *tankMsg) {
    devStdCrc32ByteArray(tankMsgHeaderAddr(tankMsg), tankMsgHeaderCrcSize(tankMsg));
    uint32_t crc32 = devStdCrc32UpdateByteArray(tankMsgDataAddr(tankMsg), tankMsgDataCrcSize(tankMsg));
    return crc32 == tankMsg->crcHeaderData;
}

CommonDataBufError tankMsgRecvBufAppendByte(uint8_t data) {
    return dataBufAppendByte(&tankMsgRecvBuf, data);
}
