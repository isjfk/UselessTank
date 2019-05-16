#include "Tank.h"
#include "system/SysIrq.h"
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
uint8_t tankPidDisableOnControlLow;

uint8_t isThrottleLow(float value);
uint8_t isYawLow(float value);

void tankInit(void) {
    tankThrottle = 0;
    tankYaw = 0;
    tankYawFixed = 0;

    tankPidSet.pid = tankPid;
    tankPidSet.size = 1;
    pidInit(&tankPidSet);

    tankPidSet.loopFreqHz = MPU_FREQ_HZ_DEFAULT;
    tankPidSet.pid[0].kP = 60;
    tankPidSet.pid[0].kI = 30;
    tankPidSet.pid[0].kD = 2;
    tankPidSet.pid[0].dNewValueWeight = 0.5;
    tankPidSet.pid[0].iLimit = 30;

    tankPidEnabled = 1;
    tankPidDisableOnControlLow = 1;
}

void tankPidLoop(void) {
    if (!tankPidEnabled
        || (tankPidDisableOnControlLow && isThrottleLow(tankThrottle) && isYawLow(tankYaw))) {
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
        tankYawFixed = tankControlRange(tankYaw + tankPidSet.pid[0].sum);
    }

    tankThrottleFixed = tankThrottle;
}

float tankControlSet(float throttle, float yaw) {
    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }

    tankThrottle = throttle;
    tankYaw = yaw;

    if (irqEnabled) {
        ei();
    }
}

float tankControlGet(float *throttle, float *yaw) {
    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }

    *throttle = tankThrottleFixed;
    *yaw = tankYawFixed;

    if (irqEnabled) {
        ei();
    }
}

float tankThrottleGet(void) {
    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }

    float throttle = tankThrottleFixed;

    if (irqEnabled) {
        ei();
    }

    return throttle;
}

void tankThrottleSet(float throttle) {
    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }

    tankThrottle = throttle;

    if (irqEnabled) {
        ei();
    }
}

float tankYawGet(void) {
    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }

    float yaw = tankYawFixed;

    if (irqEnabled) {
        ei();
    }

    return yaw;
}

void tankYawSet(float yaw) {
    uint8_t irqEnabled = isEi();
    if (irqEnabled) {
        di();
    }

    tankYaw = yaw;

    if (irqEnabled) {
        ei();
    }
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

uint8_t isThrottleLow(float value) {
    return fabs(value) < 10;
}

uint8_t isYawLow(float value) {
    return fabs(value) < 25;
}
