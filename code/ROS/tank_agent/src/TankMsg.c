#include <string.h>

#include "TankMsg.h"

static uint32_t tankMsgSeq = 1;

TankMsg* tankMsgPacketInit(TankMsgPacket *packet) {
    memset(packet, 0, sizeof(TankMsgPacket));
    packet->startTag[0] = 0x55;
    packet->startTag[1] = 0x55;
    packet->startTag[2] = 0x55;
    packet->startTag[3] = 0x55;
    packet->startTag[4] = 0x55;
    packet->startTag[5] = 0x55;
    packet->startTag[6] = 0x55;
    packet->startTag[7] = 0xFF;

    return &(packet->tankMsg);
}

uint8_t* tankMsgInit(TankMsg *tankMsg, TankMsg *tankMsgReq, uint32_t dataType, uint32_t dataLength) {
    tankMsg->desc.value = 0;
    tankMsg->desc.field.version = TANK_MSG_VERSION;
    tankMsg->desc.field.isReq = (tankMsgReq == NULL);
    tankMsg->timestampMs = 0;
    tankMsg->seq = tankMsgSeq++;
    tankMsg->seqOfReq = (tankMsgReq == NULL) ? 0 : tankMsgReq->seq;
    tankMsg->dataType = dataType;
    tankMsg->dataLength = dataLength;
    tankMsg->crcHeader = 0;
    tankMsg->crcHeaderData = 0;

    return tankMsg->data;
}
