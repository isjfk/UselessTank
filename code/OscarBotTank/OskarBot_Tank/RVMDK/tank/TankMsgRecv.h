#include <stddef.h>
#include <stdint.h>

#include "common/CommonDataBuf.h"

#ifndef __TANK_MSG_RECV_H
#define __TANK_MSG_RECV_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint32_t tankMsgRecvPrevSeq;
extern uint32_t tankMsgRecvCurrSeq;
extern uint32_t tankMsgRecvInternalErrorCount;
extern uint32_t tankMsgRecvValidMsgCount;
extern uint32_t tankMsgRecvIllegalMsgCount;
extern uint32_t tankMsgRecvUnsupportedMsgCount;

void tankMsgRecvInit(void);
void tankMsgRecvLoop(void);

CommonDataBufError tankMsgRecvBufAppendByte(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_MSG_RECV_H */
