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

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float o2_concentration;  /* uM */
    float air_saturation;    /* % */
    float temperature;       /* deg C */
    float cal_phase;         /* deg */
    float tc_phase;          /* deg */
    float c1_rph;            /* deg — blue excitation phase */
    float c2_rph;            /* deg — red excitation phase */
    float c1_amp;            /* mV  — blue excitation amplitude */
    float c2_amp;            /* mV  — red excitation amplitude */
    float raw_temp;          /* mV  — thermistor bridge voltage */
} optode_data_t;

/* Function prototypes -------------------------------------------------------*/
void optode_init(UART_HandleTypeDef *huart);
bool optode_sample(optode_data_t *out);
uint16_t optode_sample_raw(uint8_t *out, uint16_t max_len);
void optode_listen(void);

/* Split fire/collect commands for simultaneous sampling */
void optode_wake(void);
bool optode_fire(void);
bool optode_collect(optode_data_t *out);

/* Callback notify functions (called from centralized HAL callbacks) */
void optode_notify_tx_cplt(void);
void optode_notify_rx_event(uint16_t size);

#ifdef __cplusplus
}
#endif
#endif /* __OPTODE_H__ */
