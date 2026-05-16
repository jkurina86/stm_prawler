/**
  ******************************************************************************
  * @file    realtime_comm.h
  * @brief   Realtime framed CSV streaming over WiFi
  ******************************************************************************
  */
#ifndef __REALTIME_COMM_H__
#define __REALTIME_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "profile_data.h"
#include <stdbool.h>

/* Build the idata response */
void realtime_comm_build(const profile_data_t *profile);

/* Send the pre-built realtime frame to the connected WiFi TCP client. */
void realtime_comm_stream(void);

/* True while a built realtime profile is waiting to be sent. */
bool realtime_comm_data_pending(void);

#ifdef __cplusplus
}
#endif
#endif /* __REALTIME_COMM_H__ */
