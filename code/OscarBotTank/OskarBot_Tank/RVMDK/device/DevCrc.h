#include <stddef.h>
#include "stm32f10x.h"

#ifndef __DEV_CRC_H
#define __DEV_CRC_H

#ifdef __cplusplus
 extern "C" {
#endif

void devCrcInit(void);
uint32_t devCrcByteArray(uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_CRC_H */
