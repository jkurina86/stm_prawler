/**
  ******************************************************************************
  * @file    wifi.c
  * @brief   ISM4343-WBM-L54 WiFi module driver (AP mode, TCP server)
  * @note    UART4 at 115200 baud, interrupt-driven RX with ring buffer.
  *          AT command protocol: commands terminated with \r, responses
  *          terminated with "> " prompt.
  *
  *          Connection health is determined by R0 polling — there is no
  *          explicit "socket connected" query on the ISM4343. R0 returns
  *          -1 when the peer has disconnected.
  ******************************************************************************
  */

#include "wifi.h"
#include "shell.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define RX_RING_SIZE    512
#define RESP_BUF_SIZE   512
#define HEARTBEAT_INTERVAL_MS  30000   /* 30s between connection health checks */

/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t buf[RX_RING_SIZE];
    volatile uint16_t head;     /* written by ISR */
    volatile uint16_t tail;     /* read by main loop */
} ring_buffer_t;

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *wifi_huart;
static ring_buffer_t rx_ring;
static uint8_t rx_byte;        /* single-byte target for HAL_UART_Receive_IT */
static wifi_state_t state = WIFI_STATE_OFF;
static char send_buf[256];      /* shared buffer for wifi-send shell command */
static uint32_t heartbeat_tick;

/* Ring buffer helpers -------------------------------------------------------*/

static inline void rb_init(void)
{
    rx_ring.head = 0;
    rx_ring.tail = 0;
}

static inline bool rb_empty(void)
{
    return rx_ring.head == rx_ring.tail;
}

static inline void rb_push(uint8_t byte)
{
    uint16_t next = (rx_ring.head + 1) % RX_RING_SIZE;
    if (next != rx_ring.tail) {
        rx_ring.buf[rx_ring.head] = byte;
        rx_ring.head = next;
    }
}

static inline bool rb_pop(uint8_t *byte)
{
    if (rb_empty())
        return false;
    *byte = rx_ring.buf[rx_ring.tail];
    rx_ring.tail = (rx_ring.tail + 1) % RX_RING_SIZE;
    return true;
}

static inline void rb_flush(void)
{
    rx_ring.tail = rx_ring.head;
}

/* ISR callback --------------------------------------------------------------*/

void wifi_notify_rx_cplt(void)
{
    rb_push(rx_byte);
    HAL_UART_Receive_IT(wifi_huart, &rx_byte, 1);
}

/* Low-level UART helpers ----------------------------------------------------*/

static void wifi_send_str(const char *str)
{
    HAL_UART_Transmit(wifi_huart, (const uint8_t *)str, strlen(str),
                      HAL_MAX_DELAY);
}

static void wifi_send_bytes(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(wifi_huart, data, len, HAL_MAX_DELAY);
}

/**
 * @brief  Read from ring buffer until "> " prompt or timeout.
 * @return Number of bytes stored in resp_buf (null-terminated).
 */
static uint16_t wifi_read_until_prompt(char *resp_buf, uint16_t buf_size,
                                       uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint16_t idx = 0;

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t byte;
        if (rb_pop(&byte)) {
            if (idx < buf_size - 1)
                resp_buf[idx++] = (char)byte;
            /* Check for "> " prompt (last two chars) */
            if (idx >= 2 &&
                resp_buf[idx - 2] == '>' &&
                resp_buf[idx - 1] == ' ') {
                break;
            }
        }
    }
    resp_buf[idx] = '\0';
    return idx;
}

/* AT command layer ----------------------------------------------------------*/

/**
 * @brief  Send an AT command and read the response until "> " prompt.
 */
bool wifi_send_cmd(const char *cmd, char *resp_buf, uint16_t buf_size,
                   uint32_t timeout_ms)
{
    rb_flush();
    wifi_send_str(cmd);
    wifi_send_str("\r");
    wifi_read_until_prompt(resp_buf, buf_size, timeout_ms);
    return (strstr(resp_buf, "ERROR") == NULL);
}

/**
 * @brief  Send command and check for success. Logs errors via shell_printf.
 */
static bool wifi_expect_ok(const char *cmd, const char *label,
                           uint32_t timeout_ms)
{
    char resp[RESP_BUF_SIZE];
    wifi_send_cmd(cmd, resp, sizeof(resp), timeout_ms);

    if (strstr(resp, "ERROR") != NULL) {
        shell_printf("[wifi] %s failed: %s\r\n", label, resp);
        return false;
    }
    return true;
}

/* Software reset ------------------------------------------------------------*/

/**
 * @brief  Software-reset the ISM4343 via the ZR AT command.
 *         Waits 3s for module boot, then syncs with an empty command.
 */
static void wifi_soft_reset(void)
{
    char resp[RESP_BUF_SIZE];
    wifi_send_cmd("ZR", resp, sizeof(resp), 5000);
    HAL_Delay(3000);
    rb_flush();

    /* Send empty command to sync prompt */
    wifi_send_cmd("", resp, sizeof(resp), 2000);
}

/* Private helpers -----------------------------------------------------------*/

/**
 * @brief  Parse an R0 response and check connection health.
 * @param  resp      Full R0 response string.
 * @param  msg_buf   Buffer to store received data (may be NULL if not needed).
 * @param  buf_size  Size of msg_buf.
 * @return >0: bytes of data received, 0: no data (connection alive), -1: peer disconnected.
 */
static int wifi_parse_r0(const char *resp, char *msg_buf, uint16_t buf_size)
{
    char *data_start = strstr(resp, "\r\n");
    if (!data_start)
        return -1;
    data_start += 2;

    char *data_end = strstr(data_start, "\r\nOK\r\n");
    if (!data_end)
        return -1;

    uint16_t data_len = (uint16_t)(data_end - data_start);

    /* "-1" means peer disconnected */
    if (data_len == 2 && data_start[0] == '-' && data_start[1] == '1')
        return -1;

    /* Empty = no data but connection alive */
    if (data_len == 0)
        return 0;

    /* Copy received data if caller wants it */
    if (msg_buf && buf_size > 0) {
        if (data_len > buf_size - 1)
            data_len = buf_size - 1;
        memcpy(msg_buf, data_start, data_len);
        msg_buf[data_len] = '\0';
    }
    return (int)data_len;
}

/**
 * @brief  Restart TCP server after a detected disconnect.
 */
static void wifi_restart_tcp_server(void)
{
    shell_printf("[wifi] Client disconnected, restarting TCP server...\r\n");

    char resp[RESP_BUF_SIZE];
    wifi_send_cmd("P5=0", resp, sizeof(resp), 5000);

    wifi_setup_tcp_server();
}

/* Public functions ----------------------------------------------------------*/

void wifi_init(UART_HandleTypeDef *huart)
{
    wifi_huart = huart;
    rb_init();
    state = WIFI_STATE_INIT;

    /* Start single-byte interrupt reception */
    HAL_UART_Receive_IT(wifi_huart, &rx_byte, 1);

    shell_printf("[wifi] Initializing ISM4343 module...\r\n");

    if (!wifi_setup_ap()) {
        state = WIFI_STATE_ERROR;
        return;
    }

    if (!wifi_setup_tcp_server()) {
        state = WIFI_STATE_ERROR;
        return;
    }
}

wifi_state_t wifi_get_state(void)
{
    return state;
}

char *wifi_get_send_buf(void)
{
    return send_buf;
}

bool wifi_setup_ap(void)
{
    shell_printf("[wifi] Resetting module...\r\n");
    wifi_soft_reset();

    shell_printf("[wifi] Configuring Access Point...\r\n");
    if (!wifi_expect_ok("AS=0," WIFI_AP_SSID,    "Set AP SSID",    5000))
        return false;
    if (!wifi_expect_ok("A1=" WIFI_AP_SECURITY,  "Set AP Security", 5000))
        return false;
    if (!wifi_expect_ok("AC=" WIFI_AP_CHANNEL,   "Set AP Channel",  5000))
        return false;
    if (!wifi_expect_ok("Z6=" WIFI_AP_IP,        "Set AP IP",       5000))
        return false;

    shell_printf("[wifi] Starting Access Point...\r\n");
    char resp[RESP_BUF_SIZE];
    wifi_send_cmd("AD", resp, sizeof(resp), 15000);
    if (strstr(resp, "ERROR")) {
        shell_printf("[wifi] Failed to start AP: %s\r\n", resp);
        return false;
    }

    shell_printf("[wifi] AP started (SSID: %s, IP: %s)\r\n",
                 WIFI_AP_SSID, WIFI_AP_IP);
    return true;
}

bool wifi_setup_tcp_server(void)
{
    shell_printf("[wifi] Setting up TCP server on port %s...\r\n",
                 WIFI_TCP_PORT);
    if (!wifi_expect_ok("P0=0",                  "Set Socket 0",         5000))
        return false;
    if (!wifi_expect_ok("P1=0",                  "Set Protocol TCP",     5000))
        return false;
    if (!wifi_expect_ok("P2=" WIFI_TCP_PORT,     "Set Local Port",       5000))
        return false;
    if (!wifi_expect_ok("R1=1460",               "Set Read Packet Size", 5000))
        return false;
    if (!wifi_expect_ok("R2=1000",               "Set Read Timeout",     5000))
        return false;

    /* P5=1 starts listening. We don't wait for "Accepted" — instead
       we go straight to READY and let R0 heartbeats detect whether
       a client is actually connected. */
    rb_flush();
    wifi_send_str("P5=1\r");

    shell_printf("[wifi] TCP server listening on port %s\r\n", WIFI_TCP_PORT);
    state = WIFI_STATE_READY;
    heartbeat_tick = HAL_GetTick();
    return true;
}

bool wifi_send(const char *message)
{
    if (state != WIFI_STATE_READY)
        return false;

    uint16_t len = strlen(message);
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "S3=%u\r", len);

    rb_flush();
    wifi_send_str(cmd);
    HAL_Delay(50);      /* required gap before payload (per ISM4343 protocol) */
    wifi_send_bytes((const uint8_t *)message, len);

    char resp[RESP_BUF_SIZE];
    wifi_read_until_prompt(resp, sizeof(resp), 5000);

    if (strstr(resp, "ERROR")) {
        wifi_restart_tcp_server();
        return false;
    }
    return true;
}

uint16_t wifi_poll(char *msg_buf, uint16_t buf_size)
{
    if (state != WIFI_STATE_READY)
        return 0;

    char resp[RESP_BUF_SIZE];
    if (!wifi_send_cmd("R0", resp, sizeof(resp), 3000)) {
        wifi_restart_tcp_server();
        return 0;
    }

    int result = wifi_parse_r0(resp, msg_buf, buf_size);
    if (result < 0) {
        wifi_restart_tcp_server();
        return 0;
    }
    return (uint16_t)result;
}

/* Debug ---------------------------------------------------------------------*/

void wifi_dump_ring(void)
{
    shell_printf("[wifi] Ring: head=%u tail=%u\r\n", rx_ring.head, rx_ring.tail);
    uint8_t byte;
    uint16_t count = 0;
    while (rb_pop(&byte)) {
        if (byte >= 0x20 && byte < 0x7F)
            shell_printf("%c", byte);
        else
            shell_printf("\\x%02X", byte);
        count++;
    }
    if (count > 0)
        shell_printf("\r\n");
    else
        shell_printf("[wifi] Ring buffer empty\r\n");
}

/* Main-loop service ---------------------------------------------------------*/

/**
 * @brief  Non-blocking service function, call from main loop.
 *
 *  READY: sends R0 heartbeat every 30s. If R0 returns -1 or ERROR
 *         the TCP session is dead — closes socket and restarts server.
 *         If R0 returns data, prints it so it isn't silently lost.
 */
void wifi_service(void)
{
    if (state != WIFI_STATE_READY)
        return;

    if ((HAL_GetTick() - heartbeat_tick) < HEARTBEAT_INTERVAL_MS)
        return;
    heartbeat_tick = HAL_GetTick();

    char resp[RESP_BUF_SIZE];
    if (!wifi_send_cmd("R0", resp, sizeof(resp), 5000)) {
        /* R0 returned ERROR — connection is dead */
        wifi_restart_tcp_server();
        return;
    }

    char msg[256];
    int result = wifi_parse_r0(resp, msg, sizeof(msg));
    if (result < 0) {
        /* -1 means peer disconnected */
        wifi_restart_tcp_server();
        return;
    }
    if (result > 0) {
        shell_printf("[wifi] Received: %s\r\n", msg);
    }
}
