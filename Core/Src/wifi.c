/**
  ******************************************************************************
  * @file    wifi.c
  * @brief   ISM4343-WBM-L54 WiFi module driver (AP mode, TCP passthrough)
  * @note    UART4 at 9600 baud, interrupt-driven RX with ring buffer.
  *
  *          Boot sequence uses AT commands to configure AP and TCP server,
  *          then issues PX=0,0 to enter streaming mode.  After that, all
  *          UART data flows directly to/from the connected TCP client --
  *          no AT command framing.
  *
  *          wifi_service() implements a WiFi shell: assembles incoming
  *          bytes into command lines and dispatches them via the shared
  *          shell command table (shell_dispatch).
  ******************************************************************************
  */

#include "wifi.h"
#include "config.h"
#include "lowpower.h"
#include "shell.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern IWDG_HandleTypeDef hiwdg;

/* Private defines -----------------------------------------------------------*/
#define RX_RING_SIZE    512
#define RESP_BUF_SIZE   512
#define WIFI_CMD_BUF_SIZE  SHELL_MAX_CMD_LEN
#define WIFI_SHELL_PROMPT "$ "

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

/* WiFi shell command buffer */
static char wifi_cmd_buf[WIFI_CMD_BUF_SIZE];
static uint16_t wifi_cmd_pos;
static uint8_t wifi_last_eol;   /* last CR/LF byte seen, to collapse \r\n pairs */
static volatile uint32_t wifi_rx_count; /* total bytes received via ISR (diagnostic) */
static bool wifi_prompt_deferred;  /* when set, wifi_service skips the next "$ " prompt */

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

static bool wifi_command_defers_prompt(const char *cmd_line)
{
    while (*cmd_line == ' ')
        cmd_line++;

    const char *token_end = cmd_line;
    while (*token_end != '\0' && *token_end != ' ')
        token_end++;

    size_t token_len = (size_t)(token_end - cmd_line);
    return ((token_len == 5U && strncmp(cmd_line, "idata", 5U) == 0) ||
            (token_len == 3U && strncmp(cmd_line, "who", 3U) == 0));
}

static void wifi_defer_prompt(void)
{
    wifi_prompt_deferred = true;
}

/* ISR callback --------------------------------------------------------------*/

void wifi_notify_rx_cplt(void)
{
    lowpower_note_activity();
    wifi_rx_count++;
    rb_push(rx_byte);
    HAL_UART_Receive_IT(wifi_huart, &rx_byte, 1);
}

/* Low-level UART helpers ----------------------------------------------------*/

static void wifi_send_str(const char *str)
{
    HAL_UART_Transmit(wifi_huart, (const uint8_t *)str, strlen(str),
                      HAL_MAX_DELAY);
}

/**
 * @brief  Read from ring buffer until "> " prompt or timeout.
 * @return Number of bytes stored in resp_buf (null-terminated).
 */
static uint16_t wifi_read_until_prompt(char *resp_buf, uint16_t buf_size,
                                       uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t last_wdg = start;
    uint16_t idx = 0;

    while ((HAL_GetTick() - start) < timeout_ms) {
        if ((HAL_GetTick() - last_wdg) >= 200) {
            HAL_IWDG_Refresh(&hiwdg);
            last_wdg = HAL_GetTick();
        }
        uint8_t byte;
        if (rb_pop(&byte)) {
            if (idx < buf_size - 1)
                resp_buf[idx++] = (char)byte;
            /* The module normally prompts with "> ", but some firmware paths
               return a bare ">" after status/error text. */
            if ((idx >= 2 && resp_buf[idx - 2] == '>' && resp_buf[idx - 1] == ' ') ||
                resp_buf[idx - 1] == '>') {
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
 * @return Number of response bytes received (resp_buf null-terminated).
 */
static uint16_t wifi_send_cmd(const char *cmd, char *resp_buf, uint16_t buf_size,
                              uint32_t timeout_ms)
{
    rb_flush();
    wifi_send_str(cmd);
    wifi_send_str("\r");
    return wifi_read_until_prompt(resp_buf, buf_size, timeout_ms);
}

static bool wifi_resp_has_prompt(const char *resp, uint16_t len)
{
    return ((len >= 2U && resp[len - 2U] == '>' && resp[len - 1U] == ' ') ||
            (len >= 1U && resp[len - 1U] == '>'));
}

static bool wifi_resp_has_ok(const char *resp)
{
    return (strstr(resp, "OK") != NULL);
}

static bool wifi_resp_has_error(const char *resp)
{
    return (strstr(resp, "ERROR") != NULL);
}

/**
 * @brief  Send command and verify the response contains "OK" followed by a
 *         "> " prompt terminator. Logs failures via shell_printf.
 */
static bool wifi_expect_ok(const char *cmd, const char *label,
                           uint32_t timeout_ms)
{
    char resp[RESP_BUF_SIZE];
    uint16_t len = wifi_send_cmd(cmd, resp, sizeof(resp), timeout_ms);

    bool has_prompt = wifi_resp_has_prompt(resp, len);
    bool has_ok     = wifi_resp_has_ok(resp);
    bool has_error  = wifi_resp_has_error(resp);

    if (!has_prompt || !has_ok || has_error) {
        shell_printf("[wifi] %s failed: %s\r\n", label, resp);
        return false;
    }
    return true;
}

/* Setup functions -----------------------------------------------------------*/

static bool wifi_start_ap(void)
{
    shell_printf("[wifi] Starting Access Point...\r\n");
    char resp[RESP_BUF_SIZE];
    wifi_send_cmd("AD", resp, sizeof(resp), 5000);
    if (strstr(resp, "Already Running") != NULL) {
        shell_printf("[wifi] AP already running (SSID: %s, IP: %s)\r\n",
                     WIFI_AP_SSID, WIFI_AP_IP);
        return true;
    }
    if (wifi_resp_has_error(resp)) {
        shell_printf("[wifi] Failed to start AP: %s\r\n", resp);
        return false;
    }

    shell_printf("[wifi] AP started (SSID: %s, IP: %s)\r\n",
                 WIFI_AP_SSID, WIFI_AP_IP);
    return true;
}

static bool wifi_setup_ap(void)
{
    shell_printf("[wifi] Configuring Access Point...\r\n");

    if (!wifi_expect_ok("AS=0," WIFI_AP_SSID, "Set AP SSID", 2000))
        return false;
    if (!wifi_expect_ok("A1=" WIFI_AP_SECURITY, "Set AP Security",2000))
        return false;
    if (!wifi_expect_ok("AC=" WIFI_AP_CHANNEL, "Set AP Channel", 2000))
        return false;
    if (!wifi_expect_ok("Z6=" WIFI_AP_IP, "Set AP IP", 2000))
        return false;
    if (!wifi_expect_ok("ZP=1,0", "Disable Power Save", 2000))
        return false;
    if (!wifi_expect_ok("AL=255", "Set DHCP Lease Time", 2000))
        return false;

    return wifi_start_ap();
}

static bool wifi_enter_server_streaming(void)
{
    char resp[RESP_BUF_SIZE];
    wifi_send_cmd("PX=0,0", resp, sizeof(resp), 1000);

    if (wifi_resp_has_error(resp)) {
        shell_printf("[wifi] PX failed: %s\r\n", resp);
        return false;
    }

    rb_flush();
    return true;
}

static bool wifi_setup_tcp_server(void)
{
    shell_printf("[wifi] Configuring TCP passthrough on port %s...\r\n",
                 WIFI_TCP_PORT);

    if (!wifi_expect_ok("P2=" WIFI_TCP_PORT, "Set Local Port", 2000))
        return false;
    if (!wifi_expect_ok("S1=1460", "Set Write Packet Size", 2000))
        return false;
    if (!wifi_expect_ok("S2=50", "Set Write Timeout", 2000))
        return false;
    if (!wifi_expect_ok("PK=1,30000", "Enable TCP Keep-Alive", 2000))
        return false;

    /* Enter streaming mode -- this is the last AT command.  On success the
       module stays in streaming mode and does not return an AT prompt. */
    if (!wifi_enter_server_streaming()) {
        return false;
    }

    /* Flush any residual AT response bytes from the ring buffer */
    rb_flush();

    shell_printf("[wifi] TCP passthrough active on port %s\r\n", WIFI_TCP_PORT);
    state = WIFI_STATE_STREAMING;
    g_app.wifi_state = (uint8_t)state;
    return true;
}

/* Public functions ----------------------------------------------------------*/

void wifi_init(UART_HandleTypeDef *huart)
{
    wifi_huart = huart;
    rb_init();
    wifi_cmd_pos = 0;
    wifi_last_eol = 0;
    wifi_prompt_deferred = false;
    state = WIFI_STATE_INIT;
    g_app.wifi_state = (uint8_t)state;

    /* Arm RX interrupt before powering on so the module banner cannot leave
       UART4 in an overrun state before the first AT response. */
    __HAL_UART_CLEAR_OREFLAG(wifi_huart);
    HAL_UART_Receive_IT(wifi_huart, &rx_byte, 1);

    HAL_GPIO_WritePin(PB9_TRUCK_INT_OUT_GPIO_Port, PB9_TRUCK_INT_OUT_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(3000);

    rb_flush();

    shell_printf("[wifi] Initializing ISM4343 module...\r\n");

    char sync_resp[RESP_BUF_SIZE];
    wifi_send_cmd("", sync_resp, sizeof(sync_resp), 2000);
    rb_flush();

    if (!wifi_setup_ap()) {
        state = WIFI_STATE_ERROR;
        g_app.wifi_state = (uint8_t)state;
        return;
    }

    if (!wifi_setup_tcp_server()) {
        state = WIFI_STATE_ERROR;
        g_app.wifi_state = (uint8_t)state;
        return;
    }
}

wifi_state_t wifi_get_state(void)
{
    return state;
}

void wifi_down(void)
{
    if (wifi_huart != NULL)
        HAL_UART_AbortReceive_IT(wifi_huart);
    HAL_GPIO_WritePin(PB9_TRUCK_INT_OUT_GPIO_Port, PB9_TRUCK_INT_OUT_Pin,
                      GPIO_PIN_SET);
    rb_flush();
    wifi_cmd_pos = 0;
    wifi_last_eol = 0;
    wifi_prompt_deferred = false;
    state = WIFI_STATE_OFF;
    g_app.wifi_state = (uint8_t)state;
}

/* Passthrough data API ------------------------------------------------------*/

uint16_t wifi_write(const uint8_t *data, uint16_t len)
{
    if (state != WIFI_STATE_STREAMING)
        return 0;
    HAL_StatusTypeDef status = HAL_UART_Transmit(wifi_huart, data, len,
                                                  HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : 0;
}

void wifi_printf(const char *format, ...)
{
    char buffer[WIFI_PRINTF_BUF_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len > 0) {
        if ((size_t)len > sizeof(buffer) - 1)
            len = sizeof(buffer) - 1;
        wifi_write((const uint8_t *)buffer, (uint16_t)len);
    }
}

void wifi_prompt(void)
{
    wifi_write((const uint8_t *)WIFI_SHELL_PROMPT,
               (uint16_t)(sizeof(WIFI_SHELL_PROMPT) - 1U));
}

uint16_t wifi_available(void)
{
    uint16_t h = rx_ring.head;
    uint16_t t = rx_ring.tail;
    return (h >= t) ? (h - t) : (RX_RING_SIZE - t + h);
}

uint32_t wifi_get_rx_count(void)
{
    return wifi_rx_count;
}

/* Main-loop service -- WiFi shell -------------------------------------------*/

/**
 * @brief  Non-blocking WiFi shell, call from main loop.
 *
 *  Drains the RX ring buffer, assembles command lines, and dispatches
 *  them via the shared shell command table.  Sends "$ " prompt back
 *  over WiFi after each command (or empty line).
 */
void wifi_service(void)
{
    if (state != WIFI_STATE_STREAMING)
        return;

    uint8_t byte;
    while (rb_pop(&byte)) {
        switch (byte) {
        case SHELL_CHAR_CR:
        case SHELL_CHAR_LF:
            /* Collapse \r\n or \n\r into a single line ending */
            if (wifi_last_eol != 0 && byte != wifi_last_eol) {
                wifi_last_eol = 0;
                break;  /* skip the second byte of a \r\n pair */
            }
            wifi_last_eol = byte;
            wifi_cmd_buf[wifi_cmd_pos] = '\0';
            if (wifi_cmd_pos > 0) {
                if (strstr(wifi_cmd_buf, "ERROR") != NULL) {
                    /* Module rebooted and is back in AT command mode. */
                    shell_printf("[wifi] Module reset detected, reinitializing...\r\n");
                    wifi_cmd_pos = 0;
                    memset(wifi_cmd_buf, 0, sizeof(wifi_cmd_buf));
                    wifi_init(wifi_huart);
                    return;
                }
                if (wifi_command_defers_prompt(wifi_cmd_buf))
                    wifi_defer_prompt();
                lowpower_note_activity();
                shell_dispatch(wifi_cmd_buf);
                wifi_cmd_pos = 0;
                memset(wifi_cmd_buf, 0, sizeof(wifi_cmd_buf));
            }
            if (wifi_prompt_deferred)
                wifi_prompt_deferred = false;
            else
                wifi_prompt();
            break;

        case SHELL_CHAR_BS:
        case SHELL_CHAR_DEL:
            wifi_last_eol = 0;
            if (wifi_cmd_pos > 0) {
                wifi_cmd_pos--;
                wifi_cmd_buf[wifi_cmd_pos] = '\0';
            }
            break;

        case SHELL_CHAR_TAB:
        case SHELL_CHAR_ESC:
            wifi_last_eol = 0;
            break;

        default:
            wifi_last_eol = 0;
            if (byte >= 32 && byte <= 126 &&
                wifi_cmd_pos < (WIFI_CMD_BUF_SIZE - 1)) {
                wifi_cmd_buf[wifi_cmd_pos++] = (char)byte;
            }
            break;
        }
    }
}
