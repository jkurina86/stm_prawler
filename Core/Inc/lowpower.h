/**
  ******************************************************************************
  * @file    lowpower.h
  * @brief   Low-power peripheral shutdown and restore helpers
  ******************************************************************************
  */

#ifndef INC_LOWPOWER_H_
#define INC_LOWPOWER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Individual peripheral / pin-group helpers --------------------------------*/
void lowpower_sd_spi_down(void);
void lowpower_sd_spi_up(void);
bool lowpower_rtc_begin(void);
void lowpower_rtc_end(bool release_spi);

#ifdef __cplusplus
}
#endif

#endif /* INC_LOWPOWER_H_ */
