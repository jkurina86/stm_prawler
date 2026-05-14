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

/* Aggregated low-power flow -------------------------------------------------*/
void lowpower_init(void);
void lowpower_enter_idle(void);
void lowpower_note_activity(void);
void lowpower_stay_awake(void);
void lowpower_restart_timer(void);
void lowpower_idle_peripherals_down(void);
void lowpower_profile_peripherals_up(void);
void lowpower_profile_peripherals_down(void);
bool lowpower_profile_peripherals_are_up(void);
bool lowpower_rtc_begin(void);
void lowpower_rtc_end(bool release_spi);
void lowpower_wifi_start(void);
void lowpower_wifi_stop(void);
void lowpower_start_wifi_duty_cycle(void);
bool lowpower_request_on(void);
void lowpower_service(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LOWPOWER_H_ */
