#include "common/CommonDataBuf.h"

#ifndef __TANK_MSG_H
#define __TANK_MSG_H

#ifdef __cplusplus
 extern "C" {
#endif

void tankMsgInit(void);
void tankMsgLoop(void);

CommonDataBufError tankMsgReadByte(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_MSG_H */
