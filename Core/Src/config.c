/**
  ******************************************************************************
  * @file    config.c
  * @brief   Runtime system configuration
  ******************************************************************************
  */

#include "config.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t sensor_level = SENSOR_CFG_ALL;  /* default: all sensors */

/* Global application state */
app_state_t g_app;

/* Public functions ----------------------------------------------------------*/

void config_init(void)
{
    sensor_level = SENSOR_CFG_ALL;

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

bool config_has_optode(void)
{
    return sensor_level >= SENSOR_CFG_CTD_OPTODE;
}

bool config_has_wetlab(void)
{
    return sensor_level >= SENSOR_CFG_ALL;
}
