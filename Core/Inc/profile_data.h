/**
  ******************************************************************************
  * @file    profile_data.h
  * @brief   In-RAM buffer types for a complete profile recording.
  * @note    The recorder accumulates every measurement into a static
  *          profile_data_t during the profile + normalization phases, and
  *          flushes the entire CSV to SD in one pass at the end.
  ******************************************************************************
  */
#ifndef __PROFILE_DATA_H__
#define __PROFILE_DATA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "config.h"
#include "ctd.h"
#include "optode.h"
#include "wetlab.h"

/* NORM_SAMPLES mirrors the value in recorder.c. Kept in sync by PROFILE_MAX_MEASUREMENTS. */
#define PROFILE_NORM_SAMPLES       30

/* Worst-case total samples: one initial sample at PB8 trigger, plus the largest
 * profile cap (CTD-only = 315), plus the normalization phase. */
#define PROFILE_MAX_MEASUREMENTS   (1 + MEASUREMENTS_CTD_ONLY + PROFILE_NORM_SAMPLES)

typedef struct {
    uint32_t       timestamp;   /* Unix epoch UTC (seconds since Jan 1, 1970) */
    ctd_data_t     ctd;
    optode_data_t  optode;
    wetlab_data_t  wetlab;
} measurement_data_t;

typedef struct {
    uint16_t            count;
    uint32_t            start_epoch;                          /* Unix epoch at PB8 trigger */
    measurement_data_t  measurements[PROFILE_MAX_MEASUREMENTS];
} profile_data_t;

#ifdef __cplusplus
}
#endif
#endif /* __PROFILE_DATA_H__ */
