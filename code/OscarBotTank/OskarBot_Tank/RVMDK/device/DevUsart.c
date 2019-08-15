#include "DevUsart.h"

void devUsartSendByte(USART_TypeDef* usartPort, uint8_t b) {
    while (USART_GetFlagStatus(usartPort, USART_FLAG_TXE) != SET);
    USART_SendData(usartPort, b);
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
