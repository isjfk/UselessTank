#include "common/CommonDataBuf.h"

#ifndef __TANK_CMD_H
#define __TANK_CMD_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint8_t cmdInData[256];
extern CommonDataBuf cmdInBuf;

void tankCmdInit(void);
void tankCmdLoop(void);

CommonDataBufError tankCmdReadByte(uint8_t *data);
CommonDataBufError tankCmdAppendByte(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_CMD_H */
