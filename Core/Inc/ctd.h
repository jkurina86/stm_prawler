/**
  ******************************************************************************
  * @file    ctd.h
  * @brief   Driver for the CTD sensor
  * @note    This driver uses the STM32 HAL library for UART communication.
  ******************************************************************************
  */
#ifndef __CTD_H__
#define __CTD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Function prototypes -------------------------------------------------------*/
void ctd_init(UART_HandleTypeDef *huart);
bool ctd_ts(void (*print)(const char *));

#ifdef __cplusplus
}
#endif
#endif /* __CTD_H__ */
