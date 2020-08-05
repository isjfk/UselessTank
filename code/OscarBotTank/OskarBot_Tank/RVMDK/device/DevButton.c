#include "DevButton.h"

#include "system/SysTime.h"

DevButton *buttonHead = NULL;
DevButton *buttonTail = NULL;

void devButtonInit(void) {
    buttonHead = NULL;
    buttonTail = NULL;
}

void devButtonLoop(void) {
    DevButton *button = buttonHead;

    while (button != NULL) {
        int32_t currentStatus = GPIO_ReadInputDataBit(button->gpioPort, button->gpioPin);
        uint32_t currentTimeMs = sysTimeCurrentMs();

        if (currentStatus != button->prevStatus) {
            button->prevStatus = currentStatus;
            button->prevStatusSetTimeMs = currentTimeMs;
        } else if ((currentTimeMs - button->prevStatusSetTimeMs) > button->filterTimeMs) {
            button->status = currentStatus;
        }

        button = button->next;
    }
}

int32_t devButtonInitButton(DevButton *button, GPIO_TypeDef* gpioPort, uint16_t gpioPin) {
    button->gpioPort = gpioPort;
    button->gpioPin = gpioPin;
    button->level = DEV_BUTTON_LEVEL_LOW_IS_DOWN;
    button->filterTimeMs = DEV_BUTTON_FILTER_TIME_MS_DEFAULT;

    button->status = -1;
    button->prevStatus = -1;
    button->prevStatusSetTimeMs = sysTimeCurrentMs();
    button->next = NULL;

    if (buttonHead == NULL) {
        buttonHead = button;
        buttonTail = button;
    } else {
        buttonTail->next = button;
        buttonTail = button;
    }

    return 0;
}

bool devButtonIsUnknown(DevButton *button) {
    return button->status == -1;
}

bool devButtonIsDown(DevButton *button) {
    return button->status == DEV_BUTTON_STATUS_DOWN;
}

bool devButtonIsUp(DevButton *button) {
    return button->status == DEV_BUTTON_STATUS_UP;
}
