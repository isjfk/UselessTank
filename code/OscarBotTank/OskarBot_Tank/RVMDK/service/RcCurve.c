#include "RcCurve.h"

uint8_t rcCurveInit(RcCurve *curve, float (*curveData)[2], size_t curveSize) {
    curve->curveData = curveData;
    curve->curveSize = curveSize;
    return 0;
}

float rcCurveValue(RcCurve *curve, float input) {
    if (curve->curveSize == 0) {
        return input;
    }
    if (input < curve->curveData[0][0]) {
        return curve->curveData[0][1];
    }

    for (int i = 1; i < curve->curveSize; i++) {
        if (input < curve->curveData[i][0]) {
            float inStart = curve->curveData[i - 1][0];
            float inEnd = curve->curveData[i][0];
            float outStart = curve->curveData[i - 1][1];
            float outEnd = curve->curveData[i][1];
            
            return (input - inStart) / (inEnd - inStart) * (outEnd - outStart) + outStart;
        }
    }

    return curve->curveData[curve->curveSize - 1][1];
}
