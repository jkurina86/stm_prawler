/**
  ******************************************************************************
  * @file    config.c
  * @brief   Runtime system configuration
  ******************************************************************************
  */

#include "config.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t sensor_level = SENSOR_CFG_ALL;  /* default: all sensors */

/* Public functions ----------------------------------------------------------*/

void config_init(void)
{
    sensor_level = SENSOR_CFG_ALL;
}

void config_set_sensor_level(uint8_t level)
{
    if (level >= SENSOR_CFG_CTD_ONLY && level <= SENSOR_CFG_ALL)
        sensor_level = level;
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
