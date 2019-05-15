#include "service/PID.h"

#ifndef __TANK_H
#define __TANK_H

#ifdef __cplusplus
 extern "C" {
#endif

#define TANK_CTRL_MIN   (-100.f)
#define TANK_CTRL_MID   (0.f)
#define TANK_CTRL_MAX   (100.f)

extern float tankThrottle;
extern float tankThrottleFixed;
extern float tankYaw;
extern float tankYawFixed;

extern PidSet tankPidSet;
extern uint8_t tankPidEnabled;
extern uint8_t tankPidDisableOnThrottleZero;

void tankInit(void);
void tankPidLoop(void);

float tankControlSet(float throttle, float yaw);
float tankControlGet(float *throttle, float *yaw);

float tankThrottleGet(void);
void tankThrottleSet(float throttle);

float tankYawGet(void);
void tankYawSet(float yaw);

float tankControlRange(float value);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_H */
