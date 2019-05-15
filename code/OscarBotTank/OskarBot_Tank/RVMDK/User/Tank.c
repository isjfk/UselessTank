#include "Tank.h"
#include "device/DevMpu9250.h"
#include "service/Math.h"
#include "service/PID.h"

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

PID tankPid[1];
PidSet tankPidSet;
uint8_t tankPidEnabled;
uint8_t tankPidDisableOnThrottleZero;

float tankControlRange(float value);

void tankInit(void) {
    tankThrottle = 0;
    tankYaw = 0;
    tankYawFixed = 0;

    tankPidSet.pid = tankPid;
    tankPidSet.size = 1;
    pidInit(&tankPidSet);

    tankPidSet.loopFreqHz = MPU_FREQ_HZ_DEFAULT;
    tankPidSet.pid[0].kP = 100;
    tankPidSet.pid[0].kI = 30;
    tankPidSet.pid[0].kD = 2;
    tankPidSet.pid[0].dNewValueWeight = 0.1;
    tankPidSet.pid[0].iLimit = 30;

    tankPidEnabled = 1;
    tankPidDisableOnThrottleZero = 1;
}

void tankPidLoop(void) {
    if (!tankPidEnabled
        || (tankPidDisableOnThrottleZero && (tankThrottle == 0))) {
        tankYawFixed = tankYaw;
        tankThrottleFixed = tankThrottle;
        return;
    }

    float gyro[3];
    devMpu9250GetGyroFloat(gyro, NULL, NULL);

    float yawGyro = frange(gyro[2], -TANK_GYRO_YAW_MAX, TANK_GYRO_YAW_MAX);
    yawGyro = yawGyro / TANK_GYRO_YAW_MAX * TANK_CTRL_MAX;

    tankPidSet.pid[0].setPoint = tankYaw;
    tankPidSet.pid[0].measure = yawGyro;

    pidLoop(&tankPidSet);
    if (tankPidSet.updated) {
        tankYawFixed = tankControlRange(tankPidSet.pid[0].setPointFixed);
    }

    tankThrottleFixed = tankThrottle;
}

float tankControlSet(float throttle, float yaw) {
    tankThrottle = throttle;
    tankYaw = yaw;
}

float tankControlGet(float *throttle, float *yaw) {
    *throttle = tankThrottleFixed;
    *yaw = tankYawFixed;
}

float tankThrottleGet(void) {
    return tankThrottleFixed;
}

void tankThrottleSet(float throttle) {
    tankThrottle = throttle;
}

float tankYawGet(void) {
    return tankYawFixed;
}

void tankYawSet(float yaw) {
    tankYaw = yaw;
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
