#include <stdint.h>

#ifndef __SVC_PID_H
#define __SVC_PID_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
    // PID parameters.
    float kP;
    float kI;
    float kD;
    float dNewValueWeight;
    float iLimit;

    // Input value.
    float setPoint;
    float measure;

    // Output value.
    float p;
    float i;
    float d;
    float sum;
    float setPointFixed;

    // Internal data.
    float prevErrorFiltered;
} PID;

typedef struct {
    PID *pid;
    uint32_t size;

    uint32_t loopFreqHz;
    uint32_t prevSysTick;
    uint8_t updated;
} PidSet;

void pidInit(PidSet *pidSet);

void pidLoop(PidSet *pidSet);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_PID_H */
