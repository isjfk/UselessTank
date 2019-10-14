#include "PID.h"
#include "system/SysTick.h"
#include "common/CommonMath.h"

#define LOOP_FREQ_DEFAULT   100
#define PD_SCALA            100

#define K_P_DEFAULT         (60.f)
#define K_I_DEFAULT         (30.f)
#define K_D_DEFAULT         (2.f)
#define D_NEW_VALUE_WEIGHT_DEFAULT (0.5f)
#define I_LIMIT_DEFAULT     (30.f)

void pidInit(PidController *pidCtrl) {
    pidCtrl->loopFreqHz = LOOP_FREQ_DEFAULT;
    pidCtrl->prevSysTick = sysTickCurrentMs();
    pidCtrl->updated = 0;

    for (int i = 0; i < pidCtrl->size; i++) {
        PID *pid = pidCtrl->pid + i;

        pid->kP = K_P_DEFAULT;
        pid->kI = K_I_DEFAULT;
        pid->kD = K_D_DEFAULT;
        pid->dNewValueWeight = D_NEW_VALUE_WEIGHT_DEFAULT;
        pid->iLimit = I_LIMIT_DEFAULT;

        pid->setPoint = 0;
        pid->measure = 0;

        pid->p = 0;
        pid->i = 0;
        pid->d = 0;
        pid->sum = 0;

        pid->prevErrorFiltered = 0;
    }
}

void pidLoop(PidController *pidCtrl) {
    uint32_t currSysTick = sysTickCurrentMs();
    uint32_t loopFreqHz = pidCtrl->loopFreqHz;
    if ((currSysTick - pidCtrl->prevSysTick) < (1000 / loopFreqHz)) {
        pidCtrl->updated = 0;
        return;
    }

    for (int i = 0; i < pidCtrl->size; i++) {
        PID *pid = pidCtrl->pid + i;

        // PID error.
        float error = pid->setPoint - pid->measure;

        // Simple D error filter.
        float prevErrorFiltered = pid->prevErrorFiltered;
        float currErrorFiltered = prevErrorFiltered * (1 - pid->dNewValueWeight) + error * pid->dNewValueWeight;

        pid->p = error * pid->kP / PD_SCALA;
        pid->i = frange(pid->i + error * pid->kI / loopFreqHz, -pid->iLimit, pid->iLimit);
        pid->d = (currErrorFiltered - prevErrorFiltered) * pid->kD * loopFreqHz / PD_SCALA;
        pid->sum = pid->p + pid->i + pid->d;

        pid->prevErrorFiltered = currErrorFiltered;
    }

    pidCtrl->prevSysTick = currSysTick;
    pidCtrl->updated = 1;
}
