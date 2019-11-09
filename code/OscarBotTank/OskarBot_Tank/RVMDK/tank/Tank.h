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
extern float tankThrottleInput;
extern float tankYaw;
extern float tankYawInput;

extern PidController tankPidCtrl;
extern uint8_t tankPidEnabled;
extern uint8_t tankPidDisableOnControlLow;

void tankInit(void);
void tankLoop(void);
void tankPidLoop(void);

void tankControlTimeoutUpdate(void);
void tankControlTimeoutClear(void);
int tankControlIsTimeout(void);

void tankControlSet(float throttle, float yaw);
void tankControlGet(float *throttle, float *yaw);

float tankThrottleGet(void);
void tankThrottleSet(float throttle);

float tankYawGet(void);
void tankYawSet(float yaw);

float tankControlRange(float value);

float tankRcCurveThrottleValue(float input);
float tankRcCurveYawValue(float input);

#ifdef __cplusplus
}
#endif

#endif /* __TANK_H */
