#include <stddef.h>
#include <stdint.h>

#include "common/CommonMisc.h"
#include "common/CommonDataBuf.h"

#ifndef __TANK_MSG_SEND_H
#define __TANK_MSG_SEND_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint32_t tankMsgSendSuccessMsgCount;
extern uint32_t tankMsgSendOverflowMsgCount;

void tankMsgSendInit(void);
void tankMsgSendLoop(void);

CommonError tankMsgSendTextLog(uint32_t level, void *msg, size_t size);
CommonError tankMsgSendBinLog(uint32_t level, void *msg, size_t size);

CommonError tankMsgSendBufReadByte(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_MSG_SEND_H */
