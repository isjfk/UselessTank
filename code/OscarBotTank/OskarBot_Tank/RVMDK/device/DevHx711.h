#include "stdint.h"
#include "stm32f10x.h"

#ifndef __DEV_HX711_H
#define __DEV_HX711_H

#ifdef __cplusplus
extern "C" {
#endif

#define HX711_CHANNEL_A_128     1
#define HX711_CHANNEL_B_32      2
#define HX711_CHANNEL_A_64      3

void devHx711Init(void);
void devHx711On(void);
void devHx711Off(void);
void devHx711Reset(void);

/**
 * Get current channel of previous read.
 */
int32_t devHx711GetCurrentChannel(void);

/**
 * Select channel for following reads.
 * Please expect an "data not ready" read after channel changed.
 */
int32_t devHx711SelectChannel(int32_t channel);

/**
 * Read data of current selected channel.
 * Return 0 stands for data not ready.
 */
int32_t devHx711Read(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_HX711_H */
