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

void     config_init(void);
void     config_set_sensor_level(uint8_t level);
uint8_t  config_get_sensor_level(void);
bool     config_has_optode(void);
bool     config_has_wetlab(void);

#ifdef __cplusplus
}
#endif
#endif  /* CONFIG_H */
