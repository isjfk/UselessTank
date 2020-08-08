#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "common/CommonMisc.h"

#ifndef __DEV_GPIO_H
#define __DEV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get input level of a GPIO pin.
 */
static inline uint8_t devGpioBitGetInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);
}

/**
 * Check input level of a GPIO pin is high.
 */
static inline bool devGpioBitIsInputHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) > 0;
}

/**
 * Check input level of a GPIO pin is low.
 */
static inline bool devGpioBitIsInputLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0;
}

/**
 * Get output level of a GPIO pin.
 */
static inline uint8_t devGpioBitGetOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin);
}

/**
 * Check output level of a GPIO pin is high.
 */
static inline bool devGpioBitIsOutputHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) > 0;
}

/**
 * Check output level of a GPIO pin is low.
 */
static inline bool devGpioBitIsOutputLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    return GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin) == 0;
}

/**
 * Set a GPIO pin to level.
 */
static inline void devGpioBitSetLevel(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t level) {
    if (level) {
        GPIO_SetBits(GPIOx, GPIO_Pin);
    } else {
        GPIO_ResetBits(GPIOx, GPIO_Pin);
    }
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
 * Toggle level of a GPIO pin.
 */
static inline void devGpioBitToggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin)) {
        GPIO_ResetBits(GPIOx, GPIO_Pin);
    } else {
        GPIO_SetBits(GPIOx, GPIO_Pin);
    }
}



void devGpioInit(void);
void devGpioLoop(void);



typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} DevGpioPin;

typedef struct DevGpioPinPatternType {
    DevGpioPin gpioPin;
    uint8_t startLevel;
    uint8_t endLevel;
    uint32_t *patternData;
    uint32_t patternDataSize;

    uint32_t cycleSize;
    int32_t dataIndex;
    uint32_t dataDurationMs;

    struct DevGpioPinPatternType *next;
} DevGpioPinPattern;

int32_t devGpioPinPatternInit(DevGpioPinPattern *pinPattern);
bool devGpioPinPatternIsStarted(DevGpioPinPattern *pinPattern);
int32_t devGpioPinPatternStartOnce(DevGpioPinPattern *pinPattern);
int32_t devGpioPinPatternStartLoop(DevGpioPinPattern *pinPattern);
int32_t devGpioPinPatternStop(DevGpioPinPattern *pinPattern);

#define devGpioPinPatternIsPinEqual(pattern1, pattern2) (((pattern1)->gpioPin.port == (pattern2)->gpioPin.port) && ((pattern1)->gpioPin.pin == (pattern2)->gpioPin.pin))

#ifdef __cplusplus
}
#endif

#endif /* __DEV_GPIO_H */
