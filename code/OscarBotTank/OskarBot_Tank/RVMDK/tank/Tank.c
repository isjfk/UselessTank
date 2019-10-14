#include "Tank.h"
#include "TankCmd.h"
#include "TankMsg.h"
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
float tankThrottleInput;

/**
 * Tank yaw.
 *
 * Range [-100.0, 100.0].
 * 100.0: full left.
 * 0.0: go straight.
 * -100.0: full right.
 */
float tankYaw;
float tankYawInput;

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

// Level to throttle limit curve.
RcCurve level2ThrottleMinCurve;
float level2ThrottleMinCurveData[][2] = {
    { 20, TANK_CTRL_MIN },
    { 30, 0 },
};
RcCurve level2ThrottleMaxCurve;
float level2ThrottleMaxCurveData[][2] = {
    { -30, 0 },
    { -20, TANK_CTRL_MAX },
};

PID tankPid[1];
PidController tankPidCtrl;
uint8_t tankPidEnabled = 1;
uint8_t tankPidDisableOnControlLow = 1;

uint8_t isThrottleLow(float value);
uint8_t isYawLow(float value);

void tankControlInit(void);
void tankPidInit(void);

// FIXME: temp solution
void tankThrottleSlowSet() {
    float throttleInput = tankThrottleInput;
    float throttle = tankThrottle;

    // Tank throttle slow inc fast stop.
    float fullSpeedTime = 0.5;    // Slow set time in seconds.
    float step = (TANK_CTRL_MAX - TANK_CTRL_MID) / MPU_FREQ_HZ_DEFAULT / fullSpeedTime;
    if (fabsf(throttleInput - throttle) <= step) {
        throttle = throttleInput;
    } else {
        if (((throttle > 0) && (throttleInput < 0))
                || ((throttle < 0) && (throttleInput > 0))) {
            throttle = 0;
        } else if (fInRange(throttleInput, -throttle, throttle)) {
            throttle = throttleInput;
        } else {
            throttle += (throttleInput > throttle) ? step : -step;
        }
    }
    throttle = tankControlRange(throttle);

    // Limit throttle by tank level.
    float euler[3];
    devMpu9250GetEulerFloat(euler, NULL, NULL);

    float level = euler[0];
    float throttleMin = tankControlRange(rcCurveValue(&level2ThrottleMinCurve, level));
    float throttleMax = tankControlRange(rcCurveValue(&level2ThrottleMaxCurve, level));

    tankThrottle = frange(throttle, throttleMin, throttleMax);
}

void tankInit(void) {
    tankControlInit();
    tankPidInit();
    tankCmdInit();
    tankMsgInit();
}

void tankPidInit(void) {
    tankPidCtrl.pid = tankPid;
    tankPidCtrl.size = 1;
    pidInit(&tankPidCtrl);

    tankPidCtrl.loopFreqHz = MPU_FREQ_HZ_DEFAULT;
    tankPidCtrl.pid[0].kP = 25;
    tankPidCtrl.pid[0].kI = 30;
    tankPidCtrl.pid[0].kD = 2;
    tankPidCtrl.pid[0].dNewValueWeight = 0.5;
    tankPidCtrl.pid[0].iLimit = 30;
}

void tankLoop(void) {
    tankCmdLoop();

    devMpu9250Loop();
    if (devMpu9250GyroUpdated) {
        tankPidLoop();
        tankMsgLoop();
    }
}

uint32_t shouldRunTankPid(void) {
    if (!tankPidEnabled) {
        return false;
    }
    if (tankPidDisableOnControlLow && isThrottleLow(tankThrottleInput) && isYawLow(tankYawInput)) {
        return false;
    }
    return true;
}

void tankPidLoop(void) {
    float gyro[3];
    devMpu9250GetGyroFloat(gyro, NULL, NULL);

    float yawGyro = frange(gyro[2], -TANK_GYRO_YAW_MAX, TANK_GYRO_YAW_MAX);
    yawGyro = yawGyro / TANK_GYRO_YAW_MAX * TANK_CTRL_MAX;

    tankPidCtrl.pid[0].setPoint = tankYawInput;
    tankPidCtrl.pid[0].measure = shouldRunTankPid() ? yawGyro : tankYawInput;

    pidLoop(&tankPidCtrl);

    if (tankPidCtrl.updated) {
        if (!tankPidEnabled
                || (tankPidDisableOnControlLow && isThrottleLow(tankThrottleInput) && isYawLow(tankYawInput))) {
            tankYaw = tankYawInput;
        } else {
            tankYaw = tankControlRange(tankYawInput + tankPidCtrl.pid[0].sum);
        }
    }

    tankThrottleSlowSet();
}

void tankControlInit(void) {
    tankThrottleInput = 0;
    tankYaw = 0;
    tankYawInput = 0;

    rcCurveInitHelper(throttleCurve, throttleCurveData);
    rcCurveInitHelper(yawCurve, yawCurveData);

    rcCurveInitHelper(level2ThrottleMinCurve, level2ThrottleMinCurveData);
    rcCurveInitHelper(level2ThrottleMaxCurve, level2ThrottleMaxCurveData);
}

void tankControlSet(float throttle, float yaw) {
    irqLock();

    tankThrottleInput = tankControlRange(throttle);
    tankYawInput = tankControlRange(yaw);

    irqUnLock();
}

void tankControlGet(float *throttle, float *yaw) {
    irqLock();

    *throttle = tankThrottle;
    *yaw = tankYaw;

    irqUnLock();
}

float tankThrottleGet(void) {
    irqLock();

    float throttle = tankThrottle;

    irqUnLock();
    return throttle;
}

void tankThrottleSet(float throttle) {
    irqLock();

    tankThrottleInput = tankControlRange(throttle);

    irqUnLock();
}

float tankYawGet(void) {
    irqLock();

    float yaw = tankYaw;

    irqUnLock();
    return yaw;
}

void tankYawSet(float yaw) {
    irqLock();

    tankYawInput = tankControlRange(yaw);

    irqUnLock();
}

float tankControlRange(float value) {
    return frange(value, TANK_CTRL_MIN, TANK_CTRL_MAX);
}

float tankRcCurveThrottleValue(float rcValue) {
    return rcCurveValue(&throttleCurve, rcValue);
}

float tankRcCurveYawValue(float rcValue) {
    return rcCurveValue(&yawCurve, rcValue);
}

uint8_t isThrottleLow(float value) {
    return fabs(value) < 1;
}

uint8_t isYawLow(float value) {
    return fabs(value) < 1;
}
