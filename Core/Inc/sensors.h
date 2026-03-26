/**
  ******************************************************************************
  * @file    sensors.h
  * @brief   Orchestrates simultaneous sampling of CTD, Optode, and WetLab
  ******************************************************************************
  */

#ifndef SENSORS_H
#define SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ctd.h"
#include "optode.h"
#include "wetlab.h"

/* Combined sensor readings from one sample cycle */
typedef struct {
    ctd_data_t    ctd;
    optode_data_t optode;
    wetlab_data_t wetlab;
    bool          ctd_ok;
    bool          optode_ok;
    bool          wetlab_ok;
} sensor_reading_t;

/**
  * @brief  Sample all configured sensors using split-phase fire/collect
  * @param  reading: Pointer to struct that receives all sensor data and status
  * @note   Takes ~3.6s. Powers WetLab on/off. Respects config sensor level.
  */
void sensors_sample(sensor_reading_t *reading);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H */
