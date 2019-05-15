#include <stdint.h>

#ifndef __SVC_MATH_H
#define __SVC_MATH_H

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

#endif /* __SVC_MATH_H */
