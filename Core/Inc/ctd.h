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

/* Data types ----------------------------------------------------------------*/
typedef struct {
    float conductivity;
    float temperature;
    float pressure;
} ctd_data_t;

/* Function prototypes -------------------------------------------------------*/
void ctd_init(UART_HandleTypeDef *huart);
bool ctd_ts(ctd_data_t *out);

/* Split-phase API for simultaneous sampling */
bool ctd_wakeup(void);
bool ctd_fire(void);
bool ctd_collect(ctd_data_t *out);

/* Callback notify functions (called from centralized HAL callbacks) */
void ctd_notify_tx_cplt(void);
void ctd_notify_rx_event(uint16_t size);

#ifdef __cplusplus
}
#endif
#endif /* __CTD_H__ */
