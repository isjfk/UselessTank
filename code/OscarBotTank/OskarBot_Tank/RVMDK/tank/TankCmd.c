#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "TankCmd.h"
#include "Tank.h"
#include "board/Board.h"

uint8_t cmdInData[256];
CommonDataBuf cmdInBuf;

uint8_t tankCmdEnabled = 0;

#define STATE_IDLE              0
#define STATE_PREFIX            1
#define STATE_VALUE_TYPE        2
#define STATE_VALUE             3

#define VALUE_TYPE_POSITIVE     0
#define VALUE_TYPE_NEGATIVE     1

const char cmdPrefix[] = "$AP0:";
const char cmdSuffix = '!';

uint8_t tankCmdState = STATE_IDLE;
int32_t tankCmdValue = 0;
uint8_t tankCmdValueType = VALUE_TYPE_POSITIVE;

void tankCmdStateReset(void);
void tankCmdParse(uint8_t data);

void tankCmdInit(void) {
    dataBufInit(&cmdInBuf, cmdInData, sizeof(cmdInData));

    tankCmdStateReset();
    tankCmdEnabled = 1;
}

void tankCmdLoop(void) {
    uint8_t data;
    if (tankCmdReadByte(&data) != COMMON_DATABUF_OK) {
        return;
    }

    tankCmdParse(data);

    devUsartSendByte(USART1, data);
    if (data == cmdSuffix) {
        devUsartSendByte(USART1, '\r');
        devUsartSendByte(USART1, '\n');
    }
}

void tankCmdStateReset(void) {
    tankCmdState = STATE_IDLE;
    tankCmdValue = 0;
    tankCmdValueType = VALUE_TYPE_POSITIVE;
}

void tankCmdParse(uint8_t data) {
    switch (tankCmdState) {
    case STATE_IDLE:
        if (data == cmdPrefix[0]) {
            tankCmdState = STATE_PREFIX;
            tankCmdValue = 1;
        }
        break;
    case STATE_PREFIX:
        if (data == cmdPrefix[tankCmdValue]) {
            tankCmdValue++;
            if (tankCmdValue >= strlen(cmdPrefix)) {
                tankCmdState = STATE_VALUE_TYPE;
                tankCmdValue = 0;
                tankCmdValueType = VALUE_TYPE_POSITIVE;
            }
        } else {
            tankCmdStateReset();
        }
        break;
    case STATE_VALUE_TYPE:
        tankCmdState = STATE_VALUE;
        if (data == '-') {
            tankCmdValueType = VALUE_TYPE_NEGATIVE;
            break;
        } else if (!isdigit(data)) {
            tankCmdStateReset();
            break;
        }
        // Fall through for positive numbers.
    case STATE_VALUE:
        if (isdigit(data)) {
            tankCmdValue = tankCmdValue * 10 + (data - '0');
        } else if (data == cmdSuffix) {
            tankCmdStateReset();
        } else if ((data < 'A') || (data > 'Z')) {
            tankCmdStateReset();
        } else {
            if (tankCmdValueType == VALUE_TYPE_NEGATIVE) {
                tankCmdValue = -tankCmdValue;
            }

            if (data == 'X') {
                tankYawSet(tankCmdValue / 300.0);
            } else if (data == 'Y') {
                tankThrottleSet(tankCmdValue / 300.0);
            }

            tankCmdState = STATE_VALUE_TYPE;
            tankCmdValue = 0;
            tankCmdValueType = VALUE_TYPE_POSITIVE;
        }
        break;
    default:
        tankCmdStateReset();
        break;
    }
}

CommonDataBufError tankCmdReadByte(uint8_t *data) {
    if (!tankCmdEnabled) {
        return -1;
    }

    return dataBufReadByte(&cmdInBuf, data);
}

CommonDataBufError tankCmdAppendByte(uint8_t data) {
    if (!tankCmdEnabled) {
        return -1;
    }

    return dataBufAppendByte(&cmdInBuf, data);
}
