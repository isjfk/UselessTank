#include "Tank.h"
#include "TankCmd.h"
#include "common/CommonMath.h"
#include "system/SysIrq.h"
#include "device/DevMpu9250.h"
#include "service/PID.h"
#include "service/RcCurve.h"

#define TANK_GYRO_YAW_MAX           (350.f)

/**
 * Tank throttle.
 *
 * Range [-100.0, 100.0].
 * 100.0: full speed forward.
 * 0.0: stop.
 * -100.0: full speed backward.
 */
float tankThrottle;
float tankThrottleFixed;

/**
 * Tank yaw.
 *
 * Range [-100.0, 100.0].
 * 100.0: full left.
 * 0.0: go straight.
 * -100.0: full right.
 */
float tankYaw;
float tankYawFixed;

RcCurve throttleCurve;
float throttleCurveData[][2] = {
    { -100, -100 },
    { -30, 0 },
    { 30, 0 },
    { 100, 100 },
};

RcCurve yawCurve;
float yawCurveData[][2] = {
    { -100, -30 },
    { -30, 0 },
    { 30, 0 },
    { 100, 30 },
};

PID tankPid[1];
PidSet tankPidSet;
uint8_t tankPidEnabled = 1;
uint8_t tankPidDisableOnControlLow = 1;

uint8_t isThrottleLow(float value);
uint8_t isYawLow(float value);

void tankControlInit(void);
void tankPidInit(void);

// FIXME: temp solution
float tankThrottleSlowSet(float targetValue) {
    float fullSpeedTime = 1;    // seconds
    float step = (TANK_CTRL_MAX - TANK_CTRL_MID) / MPU_FREQ_HZ_DEFAULT / fullSpeedTime;
    if (tankThrottleFixed > (targetValue + step)) {
        return tankControlRange(tankThrottleFixed - step);
    } else if (tankThrottleFixed < (targetValue - step)) {
        return tankControlRange(tankThrottleFixed + step);
    } else {
        return tankControlRange(targetValue);
    }
}

void tankInit(void) {
    tankControlInit();
    tankPidInit();
    tankCmdInit();
    tankMsgInit();
}

void tankPidInit(void) {
    tankPidSet.pid = tankPid;
    tankPidSet.size = 1;
    pidInit(&tankPidSet);

    tankPidSet.loopFreqHz = MPU_FREQ_HZ_DEFAULT;
    tankPidSet.pid[0].kP = 25;
    tankPidSet.pid[0].kI = 30;
    tankPidSet.pid[0].kD = 2;
    tankPidSet.pid[0].dNewValueWeight = 0.5;
    tankPidSet.pid[0].iLimit = 30;
}

void tankLoop(void) {
    tankCmdLoop();

    devMpu9250Loop();
    if (devMpu9250GyroUpdated) {
        tankPidLoop();
        tankMsgLoop();
    }
}

void tankPidLoop(void) {
    float gyro[3];
    devMpu9250GetGyroFloat(gyro, NULL, NULL);

    float yawGyro = frange(gyro[2], -TANK_GYRO_YAW_MAX, TANK_GYRO_YAW_MAX);
    yawGyro = yawGyro / TANK_GYRO_YAW_MAX * TANK_CTRL_MAX;

    tankPidSet.pid[0].setPoint = tankYaw;
    tankPidSet.pid[0].measure = yawGyro;

    pidLoop(&tankPidSet);
    if (tankPidSet.updated) {
        if (!tankPidEnabled
                || (tankPidDisableOnControlLow && isThrottleLow(tankThrottle) && isYawLow(tankYaw))) {
            tankYawFixed = tankYaw;
        } else {
            tankYawFixed = tankControlRange(tankYaw + tankPidSet.pid[0].sum);
        }
    }

    tankThrottleFixed = tankThrottleSlowSet(tankThrottle);
}

void tankControlInit(void) {
    tankThrottle = 0;
    tankYaw = 0;
    tankYawFixed = 0;

    rcCurveInitHelper(throttleCurve, throttleCurveData);
    rcCurveInitHelper(yawCurve, yawCurveData);
}

void tankControlSet(float throttle, float yaw) {
    irqLock();

    tankThrottle = tankControlRange(throttle);
    tankYaw = tankControlRange(yaw);

    irqUnLock();
}

void tankControlGet(float *throttle, float *yaw) {
    irqLock();

    *throttle = tankThrottleFixed;
    *yaw = tankYawFixed;

    irqUnLock();
}

float tankThrottleGet(void) {
    irqLock();

    float throttle = tankThrottleFixed;

    irqUnLock();
    return throttle;
}

void tankThrottleSet(float throttle) {
    irqLock();

    tankThrottle = tankControlRange(throttle);

    irqUnLock();
}

float tankYawGet(void) {
    irqLock();

    float yaw = tankYawFixed;

    irqUnLock();
    return yaw;
}

void tankYawSet(float yaw) {
    irqLock();

    tankYaw = tankControlRange(yaw);

    irqUnLock();
}

float tankControlRange(float value) {
    if (value < TANK_CTRL_MIN) {
        return TANK_CTRL_MIN;
    }
    if (value > TANK_CTRL_MAX) {
        return TANK_CTRL_MAX;
    }
    return value;
}

float tankRcCurveThrottleValue(float input) {
    return rcCurveValue(&throttleCurve, input);
}

float tankRcCurveYawValue(float input) {
    return rcCurveValue(&yawCurve, input);
}

uint8_t isThrottleLow(float value) {
    return fabs(value) < 1;
}

uint8_t isYawLow(float value) {
    return fabs(value) < 1;
}
