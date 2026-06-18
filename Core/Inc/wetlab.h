/**
  ******************************************************************************
  * @file    wetlab.h
  * @brief   Driver for the WetLab sensor
  * @note    UART5 with DMA2 Ch2 (RX only).
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
    uint16_t ch1_lambda;  /* nm — channel 1 excitation wavelength */
    uint16_t ch1_signal;  /* counts — channel 1 */
    uint16_t ch2_lambda;  /* nm — channel 2 excitation wavelength */
    uint16_t ch2_signal;  /* counts — channel 2 */
    uint16_t ch3_lambda; /* nm — channel 3 excitation wavelength */
    uint16_t ch3_signal; /* counts — channel 3 */
    uint32_t thermistor;
} wetlab_data_t;

/* Function prototypes -------------------------------------------------------*/
void wetlab_init(UART_HandleTypeDef *huart);
bool wetlab_sample(wetlab_data_t *out);

void wetlab_raw(void);

/* Split-phase API for simultaneous sampling */
bool wetlab_fire(void);
bool wetlab_collect(wetlab_data_t *out);

#ifdef __cplusplus
}
#endif
#endif /* __WETLAB_H__ */
