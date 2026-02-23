/**
  ******************************************************************************
  * @file    wetlab.h
  * @brief   Driver for the WetLab sensor
  * @note    UART5 with DMA2 Ch1 (TX) and DMA2 Ch2 (RX).
  *          PB4 controls power to the sensor.
  ******************************************************************************
  */
#ifndef __WETLAB_H__
#define __WETLAB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t  month;
    uint8_t  day;
    uint8_t  year;        /* 2-digit */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint16_t chl_lambda;  /* nm */
    uint16_t chl_signal;
    uint16_t ntu_lambda;  /* nm */
    uint16_t ntu_signal;
    uint16_t thermistor;
} wetlab_data_t;

/* Function prototypes -------------------------------------------------------*/
void wetlab_init(UART_HandleTypeDef *huart);
bool wetlab_sample(wetlab_data_t *out);

/* Callback notify functions (called from centralized HAL callbacks) */
void wetlab_notify_tx_cplt(void);
void wetlab_notify_rx_event(uint16_t size);

#ifdef __cplusplus
}
#endif
#endif /* __WETLAB_H__ */
