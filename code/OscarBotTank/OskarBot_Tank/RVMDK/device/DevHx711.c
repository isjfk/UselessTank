#include "DevHx711.h"
#include "system/SysDelay.h"

#define hx711SetSckHigh()       GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define hx711SetSckLow()        GPIO_ResetBits(GPIOC, GPIO_Pin_9)
#define hx711GetDout()          GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)

#define HX711_CHANNEL_A_128     1
#define HX711_CHANNEL_B_32      2
#define HX711_CHANNEL_A_64      3

int32_t currChannel = HX711_CHANNEL_A_128;
int32_t nextChannel = HX711_CHANNEL_A_128;

void devHx711Init(void) {
    // Initialize HX711 IO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpio_InitStruct;
    gpio_InitStruct.GPIO_Pin = GPIO_Pin_8;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio_InitStruct);

    gpio_InitStruct.GPIO_Pin = GPIO_Pin_9;
    gpio_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio_InitStruct);

    devHx711On();
}

void devHx711On(void) {
    // Set SCK low to start HX711.
    hx711SetSckLow();
}

void devHx711Off(void) {
    // Set SCK high to power off HX711.
    hx711SetSckHigh();
}

void devHx711Reset(void) {
    devHx711Off();
    sysDelayUs(200);
    devHx711On();
}

static inline uint8_t devHx711ReadBit(void) {
    hx711SetSckHigh();
    sysDelayUs(5);
    uint8_t bit = hx711GetDout();
    hx711SetSckLow();
    sysDelayUs(5);

    return bit;
}

int32_t devHx711GetCurrentChannel(void) {
    return currChannel;
}

int32_t devHx711SelectChannel(int32_t channel) {
    nextChannel = channel;
    return currChannel;
}

/**
 * Return 0 stands for data is not ready.
 */
int32_t devHx711Read(void) {
    if (hx711GetDout()) {
        return 0;
    }

    int32_t data = 0;
    for (int i = 0; i < 24; i++) {
        data = (data << 1) | devHx711ReadBit();
    }

    switch (nextChannel) {
    case HX711_CHANNEL_A_128:
        devHx711ReadBit();
        break;
    case HX711_CHANNEL_B_32:
        devHx711ReadBit();
        devHx711ReadBit();
        break;
    case HX711_CHANNEL_A_64:
        devHx711ReadBit();
        devHx711ReadBit();
        devHx711ReadBit();
        break;
    default:
        nextChannel = HX711_CHANNEL_A_128;
        devHx711ReadBit();
        break;
    }

    if (currChannel != nextChannel) {
        data = 0;
        currChannel = nextChannel;
    } else {
        data = data << 8;
        data = data >> 8;
    }

    return data;
}
