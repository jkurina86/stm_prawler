/**
  ******************************************************************************
  * @file    wifi.h
  * @brief   ISM4343-WBM-L54 WiFi module driver (AP mode, TCP passthrough)
  * @note    UART4 at 9600 baud, interrupt-driven RX ring buffer.
  *          After boot, enters PX streaming mode -- UART becomes a
  *          transparent byte pipe to/from the connected TCP client.
  ******************************************************************************
  */
#ifndef __WIFI_H__
#define __WIFI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Configuration -------------------------------------------------------------*/
#define WIFI_AP_SSID        "prawler"
#define WIFI_AP_SECURITY    "0"             /* 0 = Open */
#define WIFI_AP_CHANNEL     "6"
#define WIFI_AP_IP          "192.168.10.1"
#define WIFI_TCP_PORT       "5000"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    WIFI_STATE_OFF,
    WIFI_STATE_INIT,
    WIFI_STATE_STREAMING,   /* AP up, TCP passthrough active */
    WIFI_STATE_ERROR
} wifi_state_t;

/* Function prototypes -------------------------------------------------------*/
void wifi_init(UART_HandleTypeDef *huart);
void wifi_down(void);
void wifi_service(void);
wifi_state_t wifi_get_state(void);

/* Passthrough data API */
uint16_t wifi_write(const uint8_t *data, uint16_t len);
void wifi_printf(const char *format, ...);
void wifi_prompt(void);
uint16_t wifi_available(void);
uint32_t wifi_get_rx_count(void);

/* Callback notify function (called from centralized HAL callback) */
void wifi_notify_rx_cplt(void);

#ifdef __cplusplus
}
#endif
#endif /* __WIFI_H__ */
