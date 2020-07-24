#include <stdint.h>
#include <stdbool.h>

#include "Tank.h"
#include "TankCmd.h"
#include "TankMsgRecv.h"
#include "TankMsgSend.h"
#include "common/CommonMath.h"
#include "system/SysIrq.h"
#include "system/SysTick.h"
#include "board/Board.h"
#include "device/DevMpu9250.h"
#include "device/DevMotor.h"
#include "device/DevHx711.h"
#include "service/PID.h"
#include "service/RcCurve.h"

#define TANK_GYRO_YAW_MAX           (350.f)

// If the tank control was not updated in timeout, stop movement of the tank.
uint32_t tankControlTimeout = 2 * 1000;
uint32_t tankControlUpdateTime = 0;

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

float hx711DataToGramScale = 1500;
#define hx711DataToGram(data)       (data / hx711DataToGramScale)

// Leash tension (in gram) to throttle limit curve.
RcCurve leashTension2ThrottleMaxCurve;
float leashTension2ThrottleMaxCurveData[][2] = {
    { 500, TANK_CTRL_MAX },
    { 1500, 0 },
};

PID tankPid[1];
PidController tankPidCtrl;
uint8_t tankPidEnabled = 1;
uint8_t tankPidDisableOnControlLow = 1;

uint8_t isThrottleLow(float value);
uint8_t isYawLow(float value);

void tankControlInit(void);
void tankPidInit(void);

void tankPidLoop(void);
void tankThrottleSlowSetLoop(void);
void tankThrottleLimitByLevelLoop(void);
void tankThrottleLimitByLeashTensionLoop(void);
void tankMotorLoop(void);

// FIXME: temp solution
void tankThrottleSlowSetLoop(void) {
    float throttleInput = tankThrottleGet();
    float throttle = tankThrottle;

    // Tank throttle slow inc fast stop.
    float fullSpeedTime = 0.5;      // Slow set time in seconds.
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

    tankThrottle = tankControlRange(throttle);
}

void tankInit(void) {
    tankControlInit();
    tankPidInit();
    tankCmdInit();
    tankMsgRecvInit();
    tankMsgSendInit();
}

void tankPidInit(void) {
    tankPidCtrl.pid = tankPid;
    tankPidCtrl.size = sizeofPidArray(tankPid);
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
    tankMsgRecvLoop();

    if (tankControlIsTimeout()) {
        tankControlSet(0, 0);
        tankControlTimeoutClear();
    }

    devMpu9250Loop();
    if (devMpu9250GyroUpdated) {
        tankPidLoop();

        tankThrottleSlowSetLoop();
        tankThrottleLimitByLevelLoop();
        tankThrottleLimitByLeashTensionLoop();

        tankMsgSendLoop();
    }

    tankMotorLoop();
}

int shouldRunTankPid(void) {
    if (!tankPidEnabled) {
        return false;
    }
    if (tankPidDisableOnControlLow && isThrottleLow(tankThrottleGet()) && isYawLow(tankYawGet())) {
        return false;
    }
    return true;
}

void tankPidLoop(void) {
    int enablePid = shouldRunTankPid();
    float gyro[3];
    devMpu9250GetGyroFloat(gyro, NULL, NULL);

    float yawInput = tankYawGet();
    float yawGyro = frange(gyro[2], -TANK_GYRO_YAW_MAX, TANK_GYRO_YAW_MAX);
    yawGyro = yawGyro / TANK_GYRO_YAW_MAX * TANK_CTRL_MAX;

    tankPidCtrl.pid[0].enabled = enablePid;
    tankPidCtrl.pid[0].setPoint = yawInput;
    tankPidCtrl.pid[0].measure = yawGyro;

    pidLoop(&tankPidCtrl);

    if (tankPidCtrl.updated) {
        tankYaw = tankControlRange(yawInput + tankPidCtrl.pid[0].sum);
    }
}

void tankThrottleLimitByLevelLoop(void) {
    float throttle = tankThrottle;

    // Limit throttle by tank level.
    float euler[3];
    devMpu9250GetEulerFloat(euler, NULL, NULL);

    float level = euler[1];
    float throttleMin = frange(rcCurveValue(&level2ThrottleMinCurve, level), TANK_CTRL_MIN, 0);
    float throttleMax = frange(rcCurveValue(&level2ThrottleMaxCurve, level), 0, TANK_CTRL_MAX);

    tankThrottle = frange(throttle, throttleMin, throttleMax);
}

void tankThrottleLimitByLeashTensionLoop(void) {
    float throttle = tankThrottle;

    int32_t hx711Data = devHx711GetData();
    float tension = hx711DataToGram(hx711Data);
    float throttleMax = frange(rcCurveValue(&leashTension2ThrottleMaxCurve, tension), 0, TANK_CTRL_MAX);

    tankThrottle = frange(throttle, TANK_CTRL_MIN, throttleMax);
}

void tankMotorLoop(void) {
    if (boardIsBatteryVeryLow()) {
        devMotorSetSpeed(0, 0);
        return;
    }

    float throttle = tankThrottle;
    float yaw = tankYaw;

    float motorSpeedLeft = frange(throttle - yaw, -100, 100);
    float motorSpeedRight = frange(throttle + yaw, -100, 100);

    devMotorSetSpeed(motorSpeedLeft, motorSpeedRight);
}

void tankControlTimeoutUpdate(void) {
    tankControlUpdateTime = sysTickCurrentMs();
}

void tankControlTimeoutClear(void) {
    tankControlUpdateTime = 0;
}

int tankControlIsTimeout(void) {
    if (tankControlUpdateTime == 0) {
        return 0;
    }

    return sysTickCurrentMs() >= (tankControlUpdateTime + tankControlTimeout);
}

void tankControlInit(void) {
    tankThrottleInput = 0;
    tankYaw = 0;
    tankYawInput = 0;

    rcCurveInitHelper(throttleCurve, throttleCurveData);
    rcCurveInitHelper(yawCurve, yawCurveData);

    rcCurveInitHelper(level2ThrottleMinCurve, level2ThrottleMinCurveData);
    rcCurveInitHelper(level2ThrottleMaxCurve, level2ThrottleMaxCurveData);

    rcCurveInitHelper(leashTension2ThrottleMaxCurve, leashTension2ThrottleMaxCurveData);
}

void tankControlSet(float throttle, float yaw) {
    irqLock();

    tankThrottleInput = tankControlRange(throttle);
    tankYawInput = tankControlRange(yaw);

    tankControlTimeoutUpdate();

    irqUnLock();
}

void tankControlGet(float *throttle, float *yaw) {
    irqLock();

    *throttle = tankThrottleInput;
    *yaw = tankYawInput;

    irqUnLock();
}

float tankThrottleGet(void) {
    irqLock();

    float throttle = tankThrottleInput;

    irqUnLock();
    return throttle;
}

void tankThrottleSet(float throttle) {
    irqLock();

    tankThrottleInput = tankControlRange(throttle);

    tankControlTimeoutUpdate();

    irqUnLock();
}

float tankYawGet(void) {
    irqLock();

    float yaw = tankYawInput;

    irqUnLock();
    return yaw;
}

void tankYawSet(float yaw) {
    irqLock();

    tankYawInput = tankControlRange(yaw);

    tankControlTimeoutUpdate();

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
