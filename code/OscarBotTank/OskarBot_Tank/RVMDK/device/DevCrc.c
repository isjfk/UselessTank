#include <string.h>

#include "DevCrc.h"

void devCrcInit(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
}

/**
 * Reset CRC checksum value.
 */
void devCrcReset(void) {
    CRC_ResetDR();
}

/**
 * Reset STM32 CRC32 checksum value.
 */
void devCrc32Reset(void) {
    CRC_ResetDR();
}

/**
 * Generate STM32 CRC32 checksum, update to current value.
 * Which is not same to commonly used CRC32 checksum in RFC1952.
 */
uint32_t devCrc32UpdateByte(uint32_t data) {
    CRC_CalcCRC(data);
}

/**
 * Generate STM32 CRC32 checksum, update to current value.
 * Which is not same to commonly used CRC32 checksum in RFC1952.
 */
uint32_t devCrc32UpdateByteArray(void *data, size_t length) {
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

/**
 * Get STM32 CRC32 checksum current value.
 */
uint32_t devCrc32Get(void) {
    return CRC_GetCRC();
}

/**
 * Generate STM32 CRC32 checksum.
 * Which is not same to commonly used CRC32 checksum in RFC1952.
 */
uint32_t devCrc32ByteArray(void *data, size_t length) {
    CRC_ResetDR();
    return devCrc32UpdateByteArray(data, length);
}

static inline uint32_t rbit(uint32_t data) {
    __asm("rbit data, data");
    return data;
}

/**
 * Reset standard CRC32 checksum value.
 */
void devStdCrc32Reset(void) {
    CRC_ResetDR();
}

/**
 * Generate standard CRC32 checksum in RFC1952, update to current value.
 */
uint32_t devStdCrc32UpdateByte(uint32_t data) {
    return rbit(CRC_CalcCRC(rbit(data))) ^ 0xFFFFFFFFL;
}

/**
 * Generate standard CRC32 checksum in RFC1952, update to current value.
 */
uint32_t devStdCrc32UpdateByteArray(void *data, size_t length) {
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

/**
 * Get standard CRC32 checksum current value.
 */
uint32_t devStdCrc32Get(void) {
    return rbit(CRC_GetCRC()) ^ 0xFFFFFFFFL;
}

/**
 * Generate standard CRC32 checksum in RFC1952.
 */
uint32_t devStdCrc32ByteArray(void *data, size_t length) {
    CRC_ResetDR();
    return devStdCrc32UpdateByteArray(data, length);
}
