/**
  ******************************************************************************
  * @file    config.c
  * @brief   Runtime system configuration
  ******************************************************************************
  */

#include "config.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t sensor_level = SENSOR_CFG_ALL;  /* default: all sensors */
static uint32_t samplerate_ms = SAMPLE_RATE_DEFAULT_MS;

/* Global application state */
app_state_t g_app;

/* Public functions ----------------------------------------------------------*/

void config_init(void)
{
    sensor_level = SENSOR_CFG_ALL;
    samplerate_ms = SAMPLE_RATE_DEFAULT_MS;

    g_app.mode          = SYS_MODE_IDLE;
    g_app.start_flag    = false;
    g_app.sd_status     = PERIPH_OFF;
    g_app.rtc_status    = PERIPH_OFF;
    g_app.ctd_status    = PERIPH_OFF;
    g_app.optode_status = PERIPH_OFF;
    g_app.wetlab_status = PERIPH_OFF;
    g_app.wifi_state    = 0;
    g_app.sensor_level  = SENSOR_CFG_ALL;
}

void config_set_sensor_level(uint8_t level)
{
    if (level >= SENSOR_CFG_CTD_ONLY && level <= SENSOR_CFG_ALL) {
        sensor_level = level;
        g_app.sensor_level = level;
    }
}

uint8_t config_get_sensor_level(void)
{
    return sensor_level;
}

uint32_t config_set_samplerate_ms(uint32_t ms)
{
    if (ms < SAMPLE_RATE_MIN_MS) {
        ms = SAMPLE_RATE_MIN_MS;
    } else if (ms > SAMPLE_RATE_MAX_MS) {
        ms = SAMPLE_RATE_MAX_MS;
    }

    samplerate_ms = ms;
    return samplerate_ms;
}

uint32_t config_get_samplerate_ms(void)
{
    return samplerate_ms;
}

bool config_has_optode(void)
{
    return sensor_level >= SENSOR_CFG_CTD_OPTODE;
}

bool config_has_wetlab(void)
{
    return sensor_level >= SENSOR_CFG_ALL;
}
