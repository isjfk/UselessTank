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

static inline float frange(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_MATH_H */
