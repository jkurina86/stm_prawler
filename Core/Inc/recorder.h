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

/* Exported function prototypes ----------------------------------------------*/
void recorder_init(void);
void recorder_service(void);
const char *recorder_get_last_filename(void);

#ifdef __cplusplus
}
#endif
#endif /* __RECORDER_H__ */
