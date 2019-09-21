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

int16_t devUsartSendData(USART_TypeDef* usartPort, int16_t data);
int16_t devUsartRecvData(USART_TypeDef* usartPort);

int devUsartSendStr(USART_TypeDef* usartPort, const char* str);

void devUsartCountInit(DevUsartCount* usartCount, USART_TypeDef* usartPort);
void devUsartCountIrq(DevUsartCount* usartCount);
void devUsartCountTx(DevUsartCount* usartCount);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_USART_H */
