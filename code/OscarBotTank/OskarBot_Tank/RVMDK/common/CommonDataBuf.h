#include <stddef.h>
#include <stdint.h>

#ifndef __COMMON_DATABUF_H
#define __COMMON_DATABUF_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef int32_t CommonDataBufError;

#define COMMON_DATABUF_OK                       (0)
#define COMMON_DATABUF_ERRNO_UNKNOWN            (-1)
#define COMMON_DATABUF_ERRNO_DATA_INSUFFICIENT  (-1001)
#define COMMON_DATABUF_ERRNO_SPACE_INSUFFICIENT (-2001)

typedef struct {
    uint8_t *buf;
    size_t bufSize;
    size_t dataStartIndex;
    size_t dataEndIndex;
    size_t dataSize;

    size_t inCount;
    size_t outCount;
    size_t overflowCount;
} CommonDataBuf;

CommonDataBufError dataBufInit(CommonDataBuf *dataBuf, uint8_t *buf, size_t bufSize);

CommonDataBufError dataBufReadByte(CommonDataBuf *dataBuf, uint8_t *data);
CommonDataBufError dataBufAppendByte(CommonDataBuf *dataBuf, uint8_t data);

CommonDataBufError dataBufReadByteArray(CommonDataBuf *dataBuf, uint8_t *data, size_t dataSize);
CommonDataBufError dataBufAppendByteArray(CommonDataBuf *dataBuf, uint8_t *data, size_t dataSize);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_DATABUF_H */
