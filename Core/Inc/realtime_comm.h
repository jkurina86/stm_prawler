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
 * cache the frame metadata.
 *
 * Frame syntax:
 *     @@@CCCCLLLL\n<payload>
 *
 * CCCC is the uppercase hex CRC, LLLL is the uppercase hex payload byte count,
 * and the newline after LLLL is part of the frame separator. The CRC covers the
 * four ASCII length bytes followed by exactly LLLL payload bytes. It excludes
 * "@@@", CCCC, and the separator newline.
 *
 * CRC algorithm: CRC-16/XMODEM-compatible, init 0x0000, polynomial 0x1021,
 * with two final zero-byte augmentations in the firmware implementation. The
 * standard "123456789" check value is 0x31C3; this is not CCITT-FALSE
 * (init 0xFFFF).
 *
 * Payload headers are fixed by an upstream contract and must not be renamed:
 *     CTD only:              EP,CD,CT,CC
 *     CTD + Optode:          EP,CD,CT,CC,OT,O2
 *     CTD + Optode + WetLab: EP,CD,CT,CC,OT,O2,CH,TB,CD
 * The short names are interpreted positionally; the final CD in the all-sensor
 * header is CDOM, not CTD depth.
 *
 * Call this once, right after a profile finishes normalizing and before
 * flushing the fat CSV to SD. */
void realtime_comm_build(const profile_data_t *profile);

/* Send the pre-built realtime frame to the connected WiFi TCP client. */
void realtime_comm_stream(void);

#ifdef __cplusplus
}
#endif
#endif /* __REALTIME_COMM_H__ */
