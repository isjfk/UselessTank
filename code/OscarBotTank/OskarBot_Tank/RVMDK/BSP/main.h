#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x.h"

// systick register
#define SYSTICK_TENMS    (*((volatile unsigned long *)0xE000E01C))  
#define SYSTICK_CURRENT  (*((volatile unsigned long *)0xE000E018))  
#define SYSTICK_RELOAD   (*((volatile unsigned long *)0xE000E014))  
#define SYSTICK_CSR      (*((volatile unsigned long *)0xE000E010)) 




#endif