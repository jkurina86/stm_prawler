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

/* Profile sample-rate limits (seconds) */
#define SAMPLE_RATE_MIN_SECONDS       4U
#define SAMPLE_RATE_MAX_SECONDS      60U
#define SAMPLE_RATE_DEFAULT_SECONDS   SAMPLE_RATE_MIN_SECONDS
#define SAMPLE_RATE_MIN_MS            (SAMPLE_RATE_MIN_SECONDS * 1000UL)
#define SAMPLE_RATE_MAX_MS            (SAMPLE_RATE_MAX_SECONDS * 1000UL)
#define SAMPLE_RATE_DEFAULT_MS        SAMPLE_RATE_MIN_MS

/* Timing constants (ms) */
#define WETLAB_BOOT_MS              1500  /* Power-on to first valid data */
#define SENSOR_COLLECT_WINDOW_MS    1200  /* DMA collection window after fire */
#define WETLAB_SAMPLE_WAIT_MS        900  /* Delay for WetLab blocking sample */
#define CTD_WAKEUP_TIMEOUT_MS         50  /* Per-attempt RX wait during wakeup */
#define CTD_WAKEUP_RETRIES            10  /* Max wakeup attempts */
#define OPTODE_WAKE_DELAY_MS         500  /* Delay after first (garbled) wake */

/* Device identity */
#define DEVICE_SERIAL              "PW002"

/* Buffer sizes */
#define CTD_RX_BUF_SIZE             1024
#define OPTODE_RX_BUF_SIZE          2048
#define WIFI_PRINTF_BUF_SIZE         256

/* Number of idata measurements per recording session */
#define IDATA_CTD_ONLY     315
#define IDATA_CTD_OPTODE   205
#define IDATA_CFG_ALL      400

/* Total number of measurements per recording session */
#define MEASUREMENTS_CTD_ONLY     315
#define MEASUREMENTS_CTD_OPTODE   205
#define MEASUREMENTS_CFG_ALL      680

/* System operating mode */
typedef enum {
    SYS_MODE_IDLE,
    SYS_MODE_RECORDING,
    SYS_MODE_FALSE_START,
    SYS_MODE_TIMEOUT,
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
    volatile bool   start_flag;      /* PB8 record trigger request */
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
uint32_t config_set_samplerate_ms(uint32_t ms);
uint32_t config_get_samplerate_ms(void);
bool     config_has_optode(void);
bool     config_has_wetlab(void);

#ifdef __cplusplus
}
#endif
#endif  /* CONFIG_H */
