#include <string.h>

#include "DevCrc.h"

void devCrcInit(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

void devCrcReset(void) {
    CRC_ResetDR();
}


/**
 * Product STM32 CRC32 checksum.
 * Which is not same to commonly used CRC32 checksum in RFC1952.
 */
uint32_t devCrc32ByteArray(void *data, size_t length) {
    uint32_t *d = (uint32_t *) data;
    size_t blockLen = length / 4;
    size_t leftDataLen = length % 4;

    uint32_t crc = CRC_CalcBlockCRC(d, blockLen);

    if (leftDataLen > 0) {
        uint32_t leftData = 0;
        memcpy(&leftData, d + blockLen, leftDataLen);
        crc = CRC_CalcCRC(leftData);
    }

    return crc;
}


uint32_t rbit(uint32_t data) {
    __asm("rbit data, data");
    return data;
}

/**
 * Produce standard CRC32 checksum in RFC1952.
 */
uint32_t devStdCrc32Update(uint32_t data) {
    return rbit(CRC_CalcCRC(rbit(data))) ^ 0xFFFFFFFFL;
}

/**
 * Produce standard CRC32 checksum in RFC1952.
 */
uint32_t devStdCrc32ByteArray(void *data, size_t length) {
    uint32_t *d = (uint32_t *) data;
    size_t blockLen = length / 4;
    size_t leftDataLen = length % 4;

    uint32_t crcReg;
    for (int i = 0; i < blockLen; i++) {
        crcReg = CRC_CalcCRC(rbit(d[i]));
    }

    if (leftDataLen > 0) {
        uint32_t leftData = 0;
        memcpy(&leftData, d + blockLen, leftDataLen);
        crcReg = CRC_CalcCRC(rbit(leftData));
    }

    return rbit(crcReg) ^ 0xFFFFFFFFL;
}
