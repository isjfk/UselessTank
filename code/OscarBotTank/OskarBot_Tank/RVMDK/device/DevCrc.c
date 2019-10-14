#include <string.h>

#include "DevCrc.h"

void devCrcInit(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

uint32_t devCrcByteArray(void *data, size_t length) {
    uint8_t *d = (uint8_t *) data;
    size_t blockLen = length / 4;
    size_t leftDataLen = length % 4;

    CRC_ResetDR();
    uint32_t crc = CRC_CalcBlockCRC((uint32_t *) d, blockLen);

    if (leftDataLen != 0) {
        uint32_t leftData = 0;
        memcpy(&leftData, d + (blockLen * 4), leftDataLen);
        crc = CRC_CalcCRC(leftData);
    }

    return crc;
}
