/**
  ******************************************************************************
  * @file    config.h
  * @brief   Configuration file for the stm_prawler
  * @note    This file contains system configuration settings.
  ******************************************************************************
  */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Device identity */
#define DEVICE_SERIAL              "PW002"

/* Peripheral status */
typedef enum {
    PERIPH_OFF,
    PERIPH_READY,
    PERIPH_ERROR
} periph_status_t;

/* Global application state */
typedef struct {
    periph_status_t sd_status;
    periph_status_t rtc_status;
    periph_status_t licor_status;
} app_state_t;

extern app_state_t g_app;
void config_init(void);

#ifdef __cplusplus
}
#endif
#endif  /* CONFIG_H */
