#include <stddef.h>

#include "common/CommonDataBuf.h"

#ifndef __TANK_MSG_SEND_H
#define __TANK_MSG_SEND_H

#ifdef __cplusplus
 extern "C" {
#endif

void tankMsgSendInit(void);
void tankMsgSendLoop(void);
CommonDataBufError tankMsgSendBufReadByte(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_MSG_SEND_H */
