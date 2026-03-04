/**
  ******************************************************************************
  * @file    recorder.h
  * @brief   PB8-triggered continuous sensor logging to SD card
  ******************************************************************************
  */
#ifndef __RECORDER_H__
#define __RECORDER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "ctd.h"
#include "optode.h"
#include "wetlab.h"

/* Exported defines ----------------------------------------------------------*/
#define RECORD_MAGIC  0xFACEFACE

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint32_t magic;
    uint32_t record_num;
    uint32_t timestamp;       /* GPS epoch seconds (since Jan 6 1980 00:00:00 UTC) */
    ctd_data_t ctd;
    optode_data_t optode;
    wetlab_data_t wetlab;
} record_data_t;

/* Exported function prototypes ----------------------------------------------*/
void recorder_init(void);
void recorder_service(void);

#ifdef __cplusplus
}
#endif
#endif /* __RECORDER_H__ */
