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

/* Individual peripheral / pin-group helpers --------------------------------*/
void lowpower_shell_uart_down(void);
void lowpower_shell_uart_up(void);

void lowpower_optode_uart_down(void);
void lowpower_optode_uart_up(void);

void lowpower_ctd_uart_down(void);
void lowpower_ctd_uart_up(void);

void lowpower_wifi_uart_down(void);
void lowpower_wifi_uart_up(void);

void lowpower_wetlab_uart_down(void);
void lowpower_wetlab_uart_up(void);

void lowpower_sd_spi_down(void);
void lowpower_sd_spi_up(void);

void lowpower_rtc_spi_down(void);
void lowpower_rtc_spi_up(void);

/* Aggregated low-power flow -------------------------------------------------*/
void lowpower_init(void);
void lowpower_idle_peripherals_down(void);
void lowpower_profile_peripherals_up(void);
void lowpower_profile_peripherals_down(void);
bool lowpower_profile_peripherals_are_up(void);
bool lowpower_rtc_begin(void);
void lowpower_rtc_end(bool release_spi);
void lowpower_wifi_start(void);
void lowpower_wifi_stop(void);
bool lowpower_request_on(void);
void lowpower_service(void);
bool lowpower_is_pending(void);
void lowpower_prepare_for_sleep(void);
void lowpower_restore_from_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LOWPOWER_H_ */
