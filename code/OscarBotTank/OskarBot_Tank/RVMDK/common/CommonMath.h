#include <stdint.h>
#include <math.h>

#ifndef __COMMON_MATH_H
#define __COMMON_MATH_H

#ifdef __cplusplus
 extern "C" {
#endif

inline int iabs(int value);
extern int iabs(int value);

inline int irange(int value, int min, int max);
extern int irange(int value, int min, int max);

inline float frange(float value, float min, float max);
extern float frange(float value, float min, float max);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_MATH_H */
