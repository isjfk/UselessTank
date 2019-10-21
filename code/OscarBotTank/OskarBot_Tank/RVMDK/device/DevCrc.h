#include <stddef.h>
#include "stm32f10x.h"

#ifndef __DEV_CRC_H
#define __DEV_CRC_H

#ifdef __cplusplus
 extern "C" {
#endif

void devCrcInit(void);
void devCrcReset(void);

uint32_t devCrc32ByteArray(void *data, size_t length);

uint32_t devStdCrc32Update(uint32_t data);
uint32_t devStdCrc32ByteArray(void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_CRC_H */
