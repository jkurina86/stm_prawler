/**
  ******************************************************************************
  * @file    optode.h
  * @brief   Driver for the Optode sensor
  * @note    This driver uses the STM32 HAL library for UART communication.
  ******************************************************************************
  */
#ifndef __OPTODE_H__
#define __OPTODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Function prototypes -------------------------------------------------------*/
void optode_init(UART_HandleTypeDef *huart);
bool optode_sample(char *buf, uint16_t buf_size);
void optode_listen(void);
void optode_setup(void);

/* Callback notify functions (called from centralized HAL callbacks) */
void optode_notify_tx_cplt(void);
void optode_notify_rx_event(uint16_t size);

#ifdef __cplusplus
}
#endif
#endif /* __OPTODE_H__ */
