#include "PID.h"
#include "system/SysTick.h"
#include "service/Math.h"

#define LOOP_FREQ_DEFAULT   100

#define K_P_DEFAULT         (100.f)
#define K_I_DEFAULT         (30.f)
#define K_D_DEFAULT         (2.f)
#define D_NEW_VALUE_WEIGHT_DEFAULT (0.1f)
#define I_LIMIT_DEFAULT     (30.f)

void pidInit(PidSet *pidSet) {
    pidSet->loopFreqHz = LOOP_FREQ_DEFAULT;
    pidSet->prevSysTick = sysTickCurrent();
    pidSet->updated = 0;

    for (int i = 0; i < pidSet->size; i++) {
        PID *pid = pidSet->pid + i;

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
        pid->setPointFixed = 0;

        pid->prevErrorFiltered = 0;
    }
}

void pidLoop(PidSet *pidSet) {
    uint32_t currSysTick = sysTickCurrent();
    uint32_t loopFreqHz = pidSet->loopFreqHz;
    if ((currSysTick - pidSet->prevSysTick) < (1000 / loopFreqHz)) {
        pidSet->updated = 0;
        return;
    }

    for (int i = 0; i < pidSet->size; i++) {
        PID *pid = pidSet->pid + i;

        // PID error.
        float error = pid->setPoint - pid->measure;

        // Simple D error filter.
        float prevErrorFiltered = pid->prevErrorFiltered;
        float currErrorFiltered = prevErrorFiltered * (1 - pid->dNewValueWeight) + error * pid->dNewValueWeight;

        pid->p = error * pid->kP / loopFreqHz;
        pid->i = frange(pid->i + error * pid->kI / loopFreqHz, -pid->iLimit, pid->iLimit);
        pid->d = (currErrorFiltered - prevErrorFiltered) * pid->kD;
        pid->sum = pid->p + pid->i + pid->d;
        pid->setPointFixed = pid->setPoint + pid->sum;

        pid->prevErrorFiltered = currErrorFiltered;
    }

    pidSet->prevSysTick = currSysTick;
    pidSet->updated = 1;
}
