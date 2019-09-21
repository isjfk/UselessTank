#include "DevUsart.h"

int16_t devUsartSendData(USART_TypeDef* usartPort, int16_t data) {
    while (USART_GetFlagStatus(usartPort, USART_FLAG_TXE) != SET);
    USART_SendData(usartPort, data);
    return data;
}

int16_t devUsartRecvData(USART_TypeDef* usartPort) {
    if (USART_GetFlagStatus(usartPort, USART_FLAG_RXNE) != SET) {
        return -1;  // EOF
    }
    return USART_ReceiveData(usartPort);
}

int devUsartSendStr(USART_TypeDef* usartPort, const char* str) {
    int count = 0;
    while (*str) {
        devUsartSendData(usartPort, *str++);
        count++;
    }
    return count;
}

void devUsartCountInit(DevUsartCount* usartCount, USART_TypeDef* usartPort) {
    usartCount->usartPort = usartPort;
    usartCount->rx = 0;
    usartCount->tx = 0;
    usartCount->irq = 0;
    usartCount->overRun = 0;
    usartCount->noise = 0;
    usartCount->framing = 0;
    usartCount->parity = 0;
}

void devUsartCountIrq(DevUsartCount* usartCount) {
    usartCount->irq++;

    if (USART_GetITStatus(usartCount->usartPort, USART_IT_RXNE) != SET) {
        return;
    }

    usartCount->rx++;
    if (USART_GetFlagStatus(usartCount->usartPort, USART_FLAG_ORE) == SET) {
        usartCount->overRun++;
    }
    if (USART_GetFlagStatus(usartCount->usartPort, USART_FLAG_NE) == SET) {
        usartCount->noise++;
    }
    if (USART_GetFlagStatus(usartCount->usartPort, USART_FLAG_FE) == SET) {
        usartCount->framing++;
    }
    if (USART_GetFlagStatus(usartCount->usartPort, USART_FLAG_PE) == SET) {
        usartCount->parity++;
    }
}

void devUsartCountTx(DevUsartCount* usartCount) {
    usartCount->tx++;
}
