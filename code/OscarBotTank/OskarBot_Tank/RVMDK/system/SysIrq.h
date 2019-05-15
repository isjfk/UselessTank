#include <stdint.h>

#include "stm32f10x.h"

#ifndef __SYS_IRQ_H
#define __SYS_IRQ_H

#ifdef __cplusplus
 extern "C" {
#endif

#define isEi()     (!__get_PRIMASK())
#define ei()        __enable_irq()
#define di()        __disable_irq()

#ifdef __cplusplus
}
#endif

#endif /* __SYS_IRQ_H */
