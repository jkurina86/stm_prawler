/**
  ******************************************************************************
  * @file    sensors.c
  * @brief   Orchestrates simultaneous sampling of CTD, Optode, and WetLab
  * @note    Split-phase fire/collect sequence. Powers WetLab on/off.
  *          Total cycle time ~3.6s.
  ******************************************************************************
  */

#include "sensors.h"
#include "main.h"
#include "config.h"
#include "stm32l4xx_hal.h"
#include <string.h>

void sensors_sample(sensor_reading_t *reading)
{
    memset(reading, 0, sizeof(*reading));

    bool has_optode = config_has_optode();
    bool has_wetlab = config_has_wetlab();

    /* 1. Power on WetLab immediately (needs ~1.5s boot) */
    if (has_wetlab)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

    /* 2. Wake CTD and Optode while WetLab boots */
    if (has_optode)
        optode_wake();
    ctd_wakeup();

    /* 3. Fire CTD and Optode */
    reading->ctd_ok = ctd_fire();
    if (has_optode)
        reading->optode_ok = optode_fire();

    /* 4. Wait for WetLab to start auto-transmitting */
    HAL_Delay(WETLAB_BOOT_MS);

    /* 5. Fire WetLab */
    if (has_wetlab)
        reading->wetlab_ok = wetlab_fire();

    /* 6. Wait for data collection */
    HAL_Delay(SENSOR_COLLECT_WINDOW_MS);

    /* 7. Collect results */
    reading->ctd_ok = ctd_collect(&reading->ctd);
    if (has_optode)
        reading->optode_ok = optode_collect(&reading->optode);
    if (has_wetlab)
        reading->wetlab_ok = wetlab_collect(&reading->wetlab);

    /* 8. Power off WetLab */
    if (has_wetlab)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}
