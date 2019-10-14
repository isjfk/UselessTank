#include <stdint.h>
#include <math.h>

#ifndef __COMMON_MATH_H
#define __COMMON_MATH_H

#ifdef __cplusplus
 extern "C" {
#endif

static inline int iabs(int value) {
    return (value < 0) ? -value : value;
}

static inline int irange(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

static inline int iInRange(int value, int edge1, int edge2) {
    int min = edge1;
    int max = edge2;
    if (edge1 > edge2) {
        min = edge2;
        max = edge1;
    }
    return (value >= min) && (value <= max);
}

static inline float frange(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

static inline int fInRange(float value, float edge1, float edge2) {
    float min = edge1;
    float max = edge2;
    if (edge1 > edge2) {
        min = edge2;
        max = edge1;
    }
    return (value >= min) && (value <= max);
}

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_MATH_H */
