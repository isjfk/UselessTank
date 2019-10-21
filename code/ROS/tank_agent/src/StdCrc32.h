#include <stddef.h>
#include <stdint.h>

#ifndef __STD_CRC32_H
#define __STD_CRC32_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    uint32_t crc;
    uint32_t dataSize;
} StdCrc32;

void stdCrc32Init(StdCrc32 *crc32);
void stdCrc32Update(StdCrc32 *crc32, void *data, size_t dataSize);
uint32_t stdCrc32Get(StdCrc32 *crc32);


#ifdef __cplusplus
}
#endif

#endif /* __STD_CRC32_H */
 
