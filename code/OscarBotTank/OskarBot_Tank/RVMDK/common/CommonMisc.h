#include <stdint.h>

#ifndef __COMMON_MISC_H
#define __COMMON_MISC_H

#ifdef __cplusplus
 extern "C" {
#endif

#define arrayLen(x)     (sizeof(x)/sizeof(x[0]))

#define COMMON_ERROR_OK                     (0)
#define COMMON_ERROR_UNKNOWN                (-1)
typedef int32_t CommonError;

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_MISC_H */
