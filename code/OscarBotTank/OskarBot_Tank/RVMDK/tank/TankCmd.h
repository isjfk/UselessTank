#include "common/CommonDataBuf.h"

#ifndef __TANK_CMD_H
#define __TANK_CMD_H

#ifdef __cplusplus
 extern "C" {
#endif

extern CommonDataBuf cmdInBuf;

void tankCmdInit(void);
void tankCmdLoop(void);

CommonDataBufError tankCmdInReadByte(uint8_t *data);
CommonDataBufError tankCmdInAppendByte(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_CMD_H */
