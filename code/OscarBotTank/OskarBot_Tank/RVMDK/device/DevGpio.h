#include <stdbool.h>
#include <stdint.h>

#include "stm32f10x.h"

#ifndef __DEV_GPIO_H
#define __DEV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get input value of a GPIO pin.
 */
static inline uint8_t devGpioBitGetInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
}

/**
 * Check input value of a GPIO pin is high.
 */
static inline bool devGpioBitIsInputHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) > 0;
}

/**
 * Check input value of a GPIO pin is low.
 */
static inline bool devGpioBitIsInputLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0;
}

/**
 * Get output value of a GPIO pin.
 */
static inline uint8_t devGpioBitGetOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin);
}

/**
 * Check output value of a GPIO pin is high.
 */
static inline bool devGpioBitIsOutputHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) > 0;
}

/**
 * Check output value of a GPIO pin is low.
 */
static inline bool devGpioBitIsOutputLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) == 0;
}

/**
 * Set a GPIO pin to high level.
 */
static inline void devGpioBitSetHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_SetBits(GPIOx, GPIO_Pin);
}

/**
 * Set a GPIO pin to low level.
 */
static inline void devGpioBitSetLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    GPIO_ResetBits(GPIOx, GPIO_Pin);
}

/**
 * Toggle a GPIO pin.
 */
static inline void devGpioBitToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
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
