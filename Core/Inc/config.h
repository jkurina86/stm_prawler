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

/* Sensor configuration levels */
#define SENSOR_CFG_CTD_ONLY       1   /* CTD */
#define SENSOR_CFG_CTD_OPTODE     2   /* CTD + Optode */
#define SENSOR_CFG_ALL            3   /* CTD + Optode + WetLab */

/* System operating mode */
typedef enum {
    SYS_MODE_IDLE,
    SYS_MODE_RECORDING,
    SYS_MODE_NORMALIZING
} sys_mode_t;

/* Peripheral status */
typedef enum {
    PERIPH_OFF,
    PERIPH_READY,
    PERIPH_ERROR
} periph_status_t;

/* Global application state */
typedef struct {
    sys_mode_t      mode;
    volatile bool   start_flag;      /* EXTI PB8 — written from ISR */
    periph_status_t sd_status;
    periph_status_t rtc_status;
    periph_status_t ctd_status;
    periph_status_t optode_status;
    periph_status_t wetlab_status;
    uint8_t         wifi_state;      /* stores wifi_state_t as uint8_t */
    uint8_t         sensor_level;
} app_state_t;

extern app_state_t g_app;

void     config_init(void);
void     config_set_sensor_level(uint8_t level);
uint8_t  config_get_sensor_level(void);
bool     config_has_optode(void);
bool     config_has_wetlab(void);

#ifdef __cplusplus
}
#endif
#endif  /* CONFIG_H */
