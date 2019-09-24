#include <string.h>

#include "DevCrc.h"

void devCrcInit(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

uint32_t devCrcByteArray(uint8_t *data, size_t length) {
    size_t blockLen = length / 4;
    size_t leftDataLen = length % 4;

    CRC_ResetDR();
    uint32_t crc = CRC_CalcBlockCRC((uint32_t *) data, blockLen);

    if (leftDataLen != 0) {
        uint32_t leftData = 0;
        memcpy(&leftData, data + (blockLen * 4), leftDataLen);
        crc = CRC_CalcCRC(leftData);
    }

    return crc;
}
