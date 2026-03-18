/**
  ******************************************************************************
  * @file    wifi.h
  * @brief   ISM4343-WBM-L54 WiFi module driver (AP mode, TCP server)
  * @note    UART4 at 115200 baud, interrupt-driven RX ring buffer.
  *          Hardware reset via PC6 (CLK_OE_Pin, active-low).
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
    WIFI_STATE_AP_READY,
    WIFI_STATE_LISTENING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_ERROR
} wifi_state_t;

/* Function prototypes -------------------------------------------------------*/
void wifi_init(UART_HandleTypeDef *huart);
wifi_state_t wifi_get_state(void);

bool wifi_setup_ap(void);
bool wifi_setup_tcp_server(void);
bool wifi_wait_for_accept(uint32_t timeout_ms);

bool wifi_send(const char *message);
uint16_t wifi_poll(char *msg_buf, uint16_t buf_size);

bool wifi_send_cmd(const char *cmd, char *resp_buf, uint16_t buf_size,
                   uint32_t timeout_ms);

char *wifi_get_send_buf(void);

/* Callback notify function (called from centralized HAL callback) */
void wifi_notify_rx_cplt(void);

#ifdef __cplusplus
}
#endif
#endif /* __WIFI_H__ */
