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

/* Build the realtime CSV payload into the module-internal RAM2 buffer and
 * cache the preamble metadata. realtime_comm_stream() sends the preamble as
 * "@@@" + CRC + length + "\n", then sends the CSV payload. The CRC still
 * covers the ASCII length field plus the CSV data. Call this once, right
 * after a profile finishes normalizing and before flushing the fat CSV to SD. */
void realtime_comm_build(const profile_data_t *profile);

/* Send the pre-built realtime frame to the connected WiFi TCP client. */
void realtime_comm_stream(void);

#ifdef __cplusplus
}
#endif
#endif /* __REALTIME_COMM_H__ */
