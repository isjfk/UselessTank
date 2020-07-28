#include <stddef.h>
#include <stdint.h>

#ifndef __TANK_MSG_H
#define __TANK_MSG_H

#ifdef __cplusplus
 extern "C" {
#endif



#define TANK_MSG_VERSION                1

typedef struct {
    union {
        struct {
            uint8_t version : 8;
            uint8_t unused1 : 8;
            uint8_t unused2 : 8;
            uint8_t isReq   : 1;
            uint8_t unused3 : 7;
        } field;
        uint32_t value;
    } desc;
    uint32_t timestamp;
    uint32_t seq;
    uint32_t seqOfReq;
    uint32_t dataType;
    uint32_t dataLength;
    uint32_t crcHeader; // CRC32 checksum of header, not include field crcHeader & crcData
    uint32_t crcData;   // CRC32 checksum of data
    uint8_t  data[100];
} TankMsg;

#define tankMsgHeaderAddr(tankMsg)      ((uint8_t *) &((tankMsg)->desc))
#define tankMsgHeaderSize(tankMsg)      (((tankMsg)->data) - ((uint8_t *) &((tankMsg)->desc)))
#define tankMsgHeaderCrcSize(tankMsg)   (((uint8_t *) &((tankMsg)->crcHeader)) - ((uint8_t *) &((tankMsg)->desc)))
#define tankMsgDataAddr(tankMsg)        ((tankMsg)->data)
#define tankMsgDataSizeMax(tankMsg)     (sizeof((tankMsg)->data))
#define tankMsgDataSize(tankMsg)        (((tankMsg)->dataLength > tankMsgDataSizeMax(tankMsg)) ? tankMsgDataSizeMax(tankMsg) : (tankMsg)->dataLength)
#define tankMsgDataCrcSize(tankMsg)     tankMsgDataSize(tankMsg)
#define tankMsgAddr(tankMsg)            ((uint8_t *) &((tankMsg)->desc))
#define tankMsgSize(tankMsg)            (tankMsgHeaderSize(tankMsg) + tankMsgDataSize(tankMsg))
#define tankMsgSet(tankMsg, offset, data) ((uint8_t *)(tankMsg))[offset] = data

#define tankMsgDataType(DataType)       (TankMsg_dataType_##DataType)
#define tankMsgDataSizeOfType(DataType) (sizeof(DataType))
#define tankMsgDataPtrOfType(tankMsg, DataType) ((DataType *) (tankMsg)->data)
#define tankMsgDataIsType(tankMsg, DataType) (tankMsg->dataType == tankMsgDataType(DataType))

#define tankMsgSizeOfType(tankMsg, DataType) (tankMsgHeaderSize(tankMsg) + tankMsgDataSizeOfType(tankMsg))

typedef struct {
    uint8_t startTag[8];
    TankMsg  tankMsg;
} TankMsgPacket;

#define tankMsgPacketAddr(packet)       ((uint8_t *) (packet))
#define tankMsgPacketSize(packet)       (sizeof((packet)->startTag) + tankMsgSize(&((packet)->tankMsg)))

typedef struct {
    float x;
    float yaw;
    float encoderTickPerMeterX;
    float encoderTickDiffFullTurnYaw;
} TankMsgCtrlTank;
#define TankMsg_dataType_TankMsgCtrlTank        1

typedef struct {
    float gyro[3];
    float accel[3];
    float compass[3];
    float quat[4];
    uint32_t motorEncoderLeft;
    uint32_t motorEncoderRight;
} TankMsgSensorData;
#define TankMsg_dataType_TankMsgSensorData      2

TankMsg* tankMsgPacketInit(TankMsgPacket *packet);

uint8_t* tankMsgInit(TankMsg *tankMsg, TankMsg *tankMsgReq, uint32_t dataType, uint32_t dataLength);
#define tankMsgReqInitByType(tankMsg, DataType)             (DataType *) tankMsgInit(tankMsg, NULL, tankMsgDataType(DataType), tankMsgDataSizeOfType(DataType))
#define tankMsgRspInitByType(tankMsg, tankMsgReq, DataType) (DataType *) tankMsgInit(tankMsg, tankMsgReq, tankMsgDataType(DataType), tankMsgDataSizeOfType(DataType))



#ifdef __cplusplus
}
#endif

#endif /* __TANK_MSG_H */
