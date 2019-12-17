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
 * Update data from HX711 on data ready.
 *
 * @return 0: data not ready loop; other: success loop, return data readed from HX711
 */
int32_t devHx711Loop(void);

/**
 * Get current selected channel.
 */
int32_t devHx711GetChannel(void);

/**
 * Select channel for following reads.
 * Data of the new channel will be ready in next data readed loop.
 */
int32_t devHx711SetChannel(int32_t channel);

/**
 * Get data of current selected channel.
 *
 * @return 0: data not ready; other: data readed from HX711 in previous success loop
 */
int32_t devHx711GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_HX711_H */
