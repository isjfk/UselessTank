#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f10x.h"

#ifndef __DEV_BUTTON_H
#define __DEV_BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#define DEV_BUTTON_LEVEL_LOW_IS_DOWN    0
#define DEV_BUTTON_LEVEL_HIGH_IS_DOWN   1

#define DEV_BUTTON_STATUS_UP            0
#define DEV_BUTTON_STATUS_DOWN          1

#define DEV_BUTTON_FILTER_TIME_MS_DEFAULT 20

typedef struct DevButtonType {
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
    uint16_t level;
    uint32_t filterTimeMs;

    int32_t status;
    int32_t prevStatus;
    uint32_t prevStatusSetTimeMs;

    struct DevButtonType *next;
} DevButton;

void devButtonInit(void);
void devButtonLoop(void);

int32_t devButtonInitButton(DevButton *button, GPIO_TypeDef* gpioPort, uint16_t gpioPin);

bool devButtonIsUnknown(DevButton *button);
bool devButtonIsDown(DevButton *button);
bool devButtonIsUp(DevButton *button);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_BUTTON_H */
