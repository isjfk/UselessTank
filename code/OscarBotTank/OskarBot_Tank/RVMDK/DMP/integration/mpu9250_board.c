#include "system/SysTick.h"
#include "mpu9250_board.h"

inline int get_tick_count(unsigned long *count)
{
    *count = sysTickMs;
    return 0;
}
