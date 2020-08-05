#include "DevGpio.h"

#include "system/SysTime.h"

static DevGpioPinPattern *pinPatternHead = NULL;
static uint32_t prevTimeMs = 0;

void devGpioPinPatternLoop(uint32_t currTimeMs, uint32_t timeDiffMs);

void devGpioInit(void) {
    pinPatternHead = NULL;
}

void devGpioLoop(void) {    
    uint32_t currTimeMs = sysTimeCurrentMs();
    uint32_t timeDiffMs = currTimeMs - prevTimeMs;
    if (timeDiffMs > 500) {
        // Handle time jump when tank first sync with ROS
        prevTimeMs = currTimeMs;
        return;
    }

    devGpioPinPatternLoop(currTimeMs, timeDiffMs);

    prevTimeMs = currTimeMs;
}

int32_t devGpioPinPatternInit(DevGpioPinPattern *pinPattern) {
    pinPattern->cycleSize = 0;
    pinPattern->dataIndex = -1;
    pinPattern->dataDurationMs = 0;

    pinPattern->next = NULL;

    return 0;
}

int32_t devGpioPinPatternStartOnce(DevGpioPinPattern *pinPattern) {
    if (pinPattern->cycleSize) {
        // Pattern already started
        if (pinPattern->cycleSize != UINT32_MAX) {
            pinPattern->cycleSize++;
        }
        return 0;
    }

    pinPattern->cycleSize = 1;
    pinPattern->dataIndex = -1;
    pinPattern->dataDurationMs = 0;

    if (pinPatternHead == NULL) {
        pinPatternHead = pinPattern;
        pinPattern->next = NULL;
        return 0;
    }

    DevGpioPinPattern *prev = NULL;
    bool prevPinMatch = false;
    for (DevGpioPinPattern *curr = pinPatternHead; curr != NULL; curr = curr->next) {
        if (devGpioPinPatternIsPinEqual(curr, pinPattern)) {
            prevPinMatch = true;
        } else {
            if (prevPinMatch) {
                // Insert after all match GPIO pins
                prev->next = pinPattern;
                pinPattern->next = curr;
                return 0;
            }
            prevPinMatch = false;
        }

        prev = curr;
    }

    prev->next = pinPattern;
    pinPattern->next = NULL;

    return 0;
}

void devGpioPinPatternLoop(uint32_t currTimeMs, uint32_t timeDiffMs) {
    if (pinPatternHead == NULL) {
        return;
    }

    DevGpioPinPattern *prev = NULL;
    DevGpioPinPattern *curr = pinPatternHead;
    while (curr != NULL) {
        if ((prev != NULL) && devGpioPinPatternIsPinEqual(curr, prev)) {
            // Don't process different pattern on same pin
            prev = curr;
            curr = curr->next;
            continue;
        }

        if (curr->dataIndex < 0) {
            // Pattern start
            curr->dataIndex = 0;
            curr->dataDurationMs = 0;

            devGpioBitSetLevel(curr->gpioPin.port, curr->gpioPin.pin, curr->startLevel);

            prev = curr;
            curr = curr->next;
            continue;
        }

        curr->dataDurationMs += timeDiffMs;
        if (curr->dataDurationMs < curr->patternData[curr->dataIndex]) {
            prev = curr;
            curr = curr->next;
            continue;
        }

        curr->dataDurationMs -= curr->patternData[curr->dataIndex];
        curr->dataIndex++;

        if (curr->dataIndex < curr->patternDataSize) {
            devGpioBitToggle(curr->gpioPin.port, curr->gpioPin.pin);

            prev = curr;
            curr = curr->next;
            continue;
        }

        curr->dataIndex = 0;
        if (curr->cycleSize != UINT32_MAX) {
            curr->cycleSize--;
        }

        if (curr->cycleSize > 0) {
            devGpioBitSetLevel(curr->gpioPin.port, curr->gpioPin.pin, curr->startLevel);

            prev = curr;
            curr = curr->next;
            continue;
        }

        // Pattern end
        curr->dataIndex = -1;
        curr->dataDurationMs = 0;

        devGpioBitSetLevel(curr->gpioPin.port, curr->gpioPin.pin, curr->endLevel);

        // Remove pattern from linked list
        if (prev == NULL) {
            // Current pattern is linked list head
            curr = curr->next;
            pinPatternHead = curr;
        } else {
            prev->next = curr->next;
            curr->next = NULL;
            curr = prev->next;
        }
    }
}
