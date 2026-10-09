#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include "stm32l4xx_hal.h"
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define GPIOB ((void *)0)
#define GPIO_PIN_8 256U
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
#endif
