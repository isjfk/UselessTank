#include <stddef.h>
#include "stm32f10x.h"

#ifndef __DEV_USART_H
#define __DEV_USART_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
    USART_TypeDef* usartPort;
    size_t rx;
    size_t tx;
    size_t irq;
    size_t overRun;
    size_t noise;
    size_t framing;
    size_t parity;
} DevUsartCount;

void devUsartSendByte(USART_TypeDef* usartPort, uint8_t b);

void devUsartCountInit(DevUsartCount* usartCount, USART_TypeDef* usartPort);
void devUsartCountIrq(DevUsartCount* usartCount);
void devUsartCountTx(DevUsartCount* usartCount);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_USART_H */
