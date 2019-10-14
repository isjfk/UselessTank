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

    // Internal data.
    float prevErrorFiltered;
} PID;

typedef struct {
    PID *pid;
    uint32_t size;

    uint32_t loopFreqHz;
    uint32_t prevSysTick;
    uint8_t updated;
} PidController;

void pidInit(PidController *pidCtrl);

void pidLoop(PidController *pidCtrl);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_PID_H */
