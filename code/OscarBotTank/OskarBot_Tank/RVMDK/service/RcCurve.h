#include <stddef.h>
#include <stdint.h>

#ifndef __SVC_RC_CURVE_H
#define __SVC_RC_CURVE_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
    float (*curveData)[2];
    size_t curveSize;
} RcCurve;

#define rcCurveInitHelper(curve, curveData)     rcCurveInit(&curve, curveData, sizeof(curveData)/sizeof(curveData[0]))
uint8_t rcCurveInit(RcCurve *curve, float (*curveData)[2], size_t curveSize);
float rcCurveValue(RcCurve *curve, float input);

#ifdef __cplusplus
}
#endif

#endif /* __SVC_RC_CURVE_H */
