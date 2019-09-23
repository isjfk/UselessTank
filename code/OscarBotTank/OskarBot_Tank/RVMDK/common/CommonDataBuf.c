#include "CommonDataBuf.h"
#include "system/SysIrq.h"

CommonDataBufError dataBufInit(CommonDataBuf *dataBuf, uint8_t *buf, size_t bufSize) {
    irqLock();

    dataBuf->buf = buf;
    dataBuf->bufSize = bufSize;
    dataBuf->dataStartIndex = 0;
    dataBuf->dataEndIndex = 0;
    dataBuf->dataSize = 0;

    dataBuf->inCount = 0;
    dataBuf->outCount = 0;
    dataBuf->overflowCount = 0;

    irqUnLock();
    return 0;
}

CommonDataBufError dataBufReadByte(CommonDataBuf *dataBuf, uint8_t *data) {
    return dataBufReadByteArray(dataBuf, data, 1);
}

CommonDataBufError dataBufAppendByte(CommonDataBuf *dataBuf, uint8_t data) {
    return dataBufAppendByteArray(dataBuf, &data, 1);
}

CommonDataBufError dataBufReadByteArray(CommonDataBuf *dataBuf, uint8_t *data, size_t dataSize) {
    irqLock();

    if (dataBuf->dataSize < dataSize) {
        irqUnLock();
        return COMMON_DATABUF_ERRNO_DATA_INSUFFICIENT;
    }

    while (dataSize-- > 0) {
        *data++ = dataBuf->buf[dataBuf->dataStartIndex++];
        dataBuf->dataStartIndex %= dataBuf->bufSize;
        dataBuf->dataSize--;

        dataBuf->outCount++;
    }

    irqUnLock();
    return 0;
}

CommonDataBufError dataBufAppendByteArray(CommonDataBuf *dataBuf, uint8_t *data, size_t dataSize) {
    irqLock();

    if ((dataBuf->bufSize - dataBuf->dataSize) < dataSize) {
        dataBuf->overflowCount++;

        irqUnLock();
        return COMMON_DATABUF_ERRNO_SPACE_INSUFFICIENT;
    }

    while (dataSize-- > 0) {
        dataBuf->buf[dataBuf->dataEndIndex++] = *data++;
        dataBuf->dataEndIndex %= dataBuf->bufSize;
        dataBuf->dataSize++;

        dataBuf->inCount++;
    }

    irqUnLock();
    return 0;
}
