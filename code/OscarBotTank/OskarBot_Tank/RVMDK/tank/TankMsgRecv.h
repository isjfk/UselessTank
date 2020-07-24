#include <stddef.h>

#include "common/CommonDataBuf.h"

#ifndef __TANK_MSG_RECV_H
#define __TANK_MSG_RECV_H

#ifdef __cplusplus
 extern "C" {
#endif

void tankMsgRecvInit(void);
void tankMsgRecvLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_MSG_RECV_H */
