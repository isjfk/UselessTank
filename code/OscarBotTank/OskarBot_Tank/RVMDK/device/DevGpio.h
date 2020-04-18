#include "stdint.h"
#include "stm32f10x.h"

#ifndef __DEV_GPIO_H
#define __DEV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Toggle a GPIO pin.
 */
static inline void devGpioToggleOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin)) {
        GPIO_ResetBits(GPIOx, GPIO_Pin);
    } else {
        GPIO_SetBits(GPIOx, GPIO_Pin);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __DEV_GPIO_H */
