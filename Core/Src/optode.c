/**
  ******************************************************************************
  * @file    optode.c
  * @brief   Driver for the Optode sensor
  * @note    This driver uses the STM32 HAL library for UART communication.
  *          USART2 with DMA1 Ch7 (TX) and DMA1 Ch6 (RX).
  ******************************************************************************
  */

#include "optode.h"
#include "config.h"
#include "filesystem.h"
#include "ab-rtcmc-rtc.h"
#include "shell.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *optode_huart;
static volatile bool tx_done = false;
static volatile bool rx_done = false;
static volatile uint16_t rx_len = 0;
static uint8_t rx_buf[OPTODE_RX_BUF_SIZE];

/*
 * Wake-up preamble: '/' comment characters
 * followed by CR+LF to terminate the comment line.  The sensor ignores
 * comment lines (no response), but the incoming bytes wake it from
 * Communication Sleep.  See datasheet section 5.6.
 */
static const char wake_preamble[] = "////////\r\n";

/* Callback notify functions (called from centralized HAL callbacks) ----------*/

void optode_notify_tx_cplt(void)
{
    tx_done = true;
}

void optode_notify_rx_event(uint16_t size)
{
    rx_len = size;
    rx_done = true;
}

/* Private functions ---------------------------------------------------------*/

static void optode_reset_uart(void)
{
    HAL_UART_Abort(optode_huart);
    __HAL_UART_CLEAR_OREFLAG(optode_huart);
    __HAL_UART_CLEAR_NEFLAG(optode_huart);
    __HAL_UART_CLEAR_FEFLAG(optode_huart);
}

static bool optode_arm_rx(uint8_t *buf, uint16_t len)
{
    rx_done = false;
    rx_len = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(optode_huart, buf, len) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(optode_huart->hdmarx, DMA_IT_HT);
    return true;
}

/* Public functions ----------------------------------------------------------*/

void optode_init(UART_HandleTypeDef *huart)
{
    optode_huart = huart;
}

/**
 * @brief  Send a buffer via DMA TX and spin-wait for completion.
 * @return true if TX completed, false on timeout.
 */
static bool optode_tx(const uint8_t *data, uint16_t len)
{
    tx_done = false;
    if (HAL_UART_Transmit_DMA(optode_huart, data, len) != HAL_OK)
        return false;
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 2000) {
            HAL_UART_AbortTransmit(optode_huart);
            return false;
        }
    }
    return true;
}

/**
 * @brief  Send preamble + command once and collect until '#' or '*'.
 * @return Number of bytes in rx_buf, or 0 on failure.
 */
static uint16_t optode_send_sample(bool with_preamble)
{
    const char *cmd = "do sample\r\n";
    uint32_t t0;
    uint16_t total = 0;

    optode_reset_uart();

    if (!optode_arm_rx(rx_buf, sizeof(rx_buf) - 1))
        return 0;

    if (with_preamble) {
        if (!optode_tx((const uint8_t *)wake_preamble,
                       sizeof(wake_preamble) - 1)) {
            HAL_UART_Abort(optode_huart);
            return 0;
        }
    }

    if (!optode_tx((const uint8_t *)cmd, strlen(cmd))) {
        HAL_UART_Abort(optode_huart);
        return 0;
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 10000 && total < sizeof(rx_buf) - 1) {
        if (rx_done) {
            if (rx_len > 0) {
                total += rx_len;
                if (memchr(rx_buf + total - rx_len, '#', rx_len))
                    break;
                if (memchr(rx_buf + total - rx_len, '*', rx_len))
                    break;
            }
            uint16_t remaining = sizeof(rx_buf) - 1 - total;
            if (remaining == 0)
                break;
            if (!optode_arm_rx(rx_buf + total, remaining))
                break;
        }
    }
    HAL_UART_AbortReceive(optode_huart);

    rx_buf[total] = '\0';
    return total;
}

/**
 * @brief  Parse a tab-separated measurement line into an optode_data_t.
 *
 * Current sensor config (Enable AirSaturation=No, Enable Rawdata=Yes):
 *   4330\t1638\tO2Conc\tTemp\tCalPhase\tTCPhase\tC1RPh\tC2RPh\tC1Amp\tC2Amp\tRawTemp
 * With Enable Text=No the labels are omitted; field order is the same.
 *
 * Strategy: first two tab tokens are product/serial (integers) — skip them.
 * Remaining tokens: try strtof(); skip text labels (endptr == token).
 * Assign first two successful floats to O2 concentration and temperature.
 */
static bool optode_parse(char *line, optode_data_t *out)
{
    /* Skip product_no token */
    char *tok = strtok(line, "\t");
    if (!tok) return false;

    /* Skip serial_no token */
    tok = strtok(NULL, "\t");
    if (!tok) return false;

    int idx = 0;
    float vals[9];
    while ((tok = strtok(NULL, "\t")) != NULL && idx < 9) {
        char *end;
        float v = strtof(tok, &end);
        if (end == tok)
            continue;   /* text label — skip */
        vals[idx++] = v;
    }
    if (idx < 9)
        return false;

    out->o2_concentration = vals[0];
    out->temperature      = vals[1];
    out->cal_phase        = vals[2];
    out->tc_phase         = vals[3];
    out->c1_rph           = vals[4];
    out->c2_rph           = vals[5];
    out->c1_amp           = vals[6];
    out->c2_amp           = vals[7];
    out->raw_temp         = vals[8];
    return true;
}

bool optode_sample(optode_data_t *out)
{
    /* First attempt: preamble wakes sensor from Communication Sleep */
    uint16_t total = optode_send_sample(true);

    /*
     * If the sensor's UART was unsynchronized on wake, it returns
     * '* ERROR SYNTAX ERROR' from garbled preamble bytes.  The sensor
     * is now awake, so retry the command without the preamble.
     */
    if (total > 0 && memchr(rx_buf, '*', total) && !memchr(rx_buf, '#', total))
        total = optode_send_sample(false);

    if (total == 0)
        return false;

    /* Find the first digit in the buffer (start of product number) */
    char *p = (char *)rx_buf;
    while (*p && (*p < '0' || *p > '9'))
        p++;
    if (!*p)
        return false;

    return optode_parse(p, out);
}

/* Split-phase API for simultaneous sampling --------------------------------*/

void optode_wake(void)
{
    const uint16_t buf_size = sizeof(rx_buf) - 1;

    optode_reset_uart();

    /* Arm RX DMA in polling mode */
    if (HAL_UART_Receive_DMA(optode_huart, rx_buf, buf_size) != HAL_OK)
        return;
    __HAL_DMA_DISABLE_IT(optode_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(optode_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(optode_huart->Instance->CR3, USART_CR3_EIE);

    /* TX wake preamble */
    tx_done = false;
    if (HAL_UART_Transmit_DMA(optode_huart,
                               (uint8_t *)wake_preamble,
                               sizeof(wake_preamble) - 1) != HAL_OK) {
        HAL_UART_Abort(optode_huart);
        return;
    }
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 2000) {
            HAL_UART_Abort(optode_huart);
            return;
        }
    }

    /* Wait up to 500 ms for any error response, then discard */
    HAL_Delay(OPTODE_WAKE_DELAY_MS);
    HAL_UART_AbortReceive(optode_huart);
}

bool optode_fire(void)
{
    const char *cmd = "do sample\r\n";
    const uint16_t buf_size = sizeof(rx_buf) - 1;

    optode_reset_uart();

    /* Start continuous RX DMA — poll NDTR, no callbacks */
    if (HAL_UART_Receive_DMA(optode_huart, rx_buf, buf_size) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(optode_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(optode_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(optode_huart->Instance->CR3, USART_CR3_EIE);

    /* TX command via DMA */
    tx_done = false;
    if (HAL_UART_Transmit_DMA(optode_huart,
                               (uint8_t *)cmd, strlen(cmd)) != HAL_OK) {
        HAL_UART_AbortReceive(optode_huart);
        return false;
    }
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 2000) {
            HAL_UART_Abort(optode_huart);
            return false;
        }
    }
    return true;
}

bool optode_collect(optode_data_t *out)
{
    const uint16_t buf_size = sizeof(rx_buf) - 1;
    uint16_t total = buf_size - __HAL_DMA_GET_COUNTER(optode_huart->hdmarx);
    HAL_UART_AbortReceive(optode_huart);

    if (total == 0)
        return false;

    rx_buf[total] = '\0';

    /* Find the first digit in the buffer (start of product number) */
    char *p = (char *)rx_buf;
    while (*p && (*p < '0' || *p > '9'))
        p++;
    if (!*p)
        return false;

    return optode_parse(p, out);
}

/**
 * @brief  Send a command string and wait for '#' (OK) or '*' (error).
 * @return true if '#' was received, false on '*' or timeout.
 */
static bool optode_send_cmd(const char *cmd, uint32_t timeout_ms)
{
    uint16_t total = 0;

    optode_reset_uart();
    if (!optode_arm_rx(rx_buf, sizeof(rx_buf) - 1))
        return false;

    tx_done = false;
    if (HAL_UART_Transmit_DMA(optode_huart,
                               (uint8_t *)cmd, strlen(cmd)) != HAL_OK) {
        HAL_UART_AbortReceive(optode_huart);
        return false;
    }

    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_Abort(optode_huart);
            return false;
        }
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms && total < sizeof(rx_buf) - 1) {
        if (rx_done) {
            if (rx_len > 0) {
                total += rx_len;
                if (memchr(rx_buf + total - rx_len, '#', rx_len)) {
                    HAL_UART_AbortReceive(optode_huart);
                    rx_buf[total] = '\0';
                    return true;
                }
                if (memchr(rx_buf + total - rx_len, '*', rx_len)) {
                    HAL_UART_AbortReceive(optode_huart);
                    rx_buf[total] = '\0';
                    return false;
                }
            }
            uint16_t remaining = sizeof(rx_buf) - 1 - total;
            if (remaining == 0)
                break;
            if (!optode_arm_rx(rx_buf + total, remaining))
                break;
        }
    }
    HAL_UART_AbortReceive(optode_huart);
    return false;
}

void optode_setup(void)
{
    const uint32_t sensor_baud = 4800;

    shell_printf("[optode] Power-cycling at %lu baud...\r\n", sensor_baud);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(2000);

    /* Switch USART2 to the sensor's current baud rate */
    optode_huart->Init.BaudRate = sensor_baud;
    if (HAL_UART_Init(optode_huart) != HAL_OK) {
        shell_printf("[optode] UART init failed\r\n");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        return;
    }

    /* Power on and wait for startup + sleep */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(3000);

    /* Wake the sensor with comment-line preamble */
    shell_printf("[optode] Waking sensor...\r\n");
    optode_reset_uart();
    if (!optode_tx((const uint8_t *)wake_preamble, sizeof(wake_preamble) - 1)) {
        shell_printf("[optode] Wake TX failed\r\n");
        goto restore;
    }

    /* Unlock write-protected properties (High protection = passkey 1000) */
    shell_printf("[optode] Set Passkey(1000)...\r\n");
    if (optode_send_cmd("set passkey(1000)\r\n", 5000))
        shell_printf("[optode] OK\r\n");
    else
        shell_printf("[optode] Failed (response: %s)\r\n", (char *)rx_buf);

    /* Set mode to Smart Sensor Terminal */
    shell_printf("[optode] Set Mode(Smart Sensor Terminal)...\r\n");
    if (optode_send_cmd("set mode(Smart Sensor Terminal)\r\n", 5000))
        shell_printf("[optode] OK\r\n");
    else
        shell_printf("[optode] Failed (response: %s)\r\n", (char *)rx_buf);

    /* Set baud rate to 9600 */
    shell_printf("[optode] Set Baudrate(9600)...\r\n");
    if (optode_send_cmd("set baudrate(9600)\r\n", 5000))
        shell_printf("[optode] OK\r\n");
    else
        shell_printf("[optode] Failed (response: %s)\r\n", (char *)rx_buf);

    /* Save to flash (can take up to 20 seconds per the datasheet) */
    shell_printf("[optode] Save (may take up to 20s)...\r\n");
    if (optode_send_cmd("save\r\n", 25000))
        shell_printf("[optode] OK\r\n");
    else
        shell_printf("[optode] Failed (response: %s)\r\n", (char *)rx_buf);

    /* Reset the sensor so new settings take effect */
    shell_printf("[optode] Reset...\r\n");
    optode_send_cmd("reset\r\n", 5000);

restore:
    /* Restore USART2 to 9600 */
    optode_huart->Init.BaudRate = 9600;
    HAL_UART_Init(optode_huart);
    shell_printf("[optode] USART2 restored to 9600. "
                 "Power-cycle the sensor to verify.\r\n");
}

void optode_listen(void)
{
    static const uint32_t bauds[] = {9600, 19200, 38400, 57600, 4800, 115200};
    static const int nbauds = sizeof(bauds) / sizeof(bauds[0]);

    for (int i = 0; i < nbauds; i++) {
        shell_printf("\r\n[optode] --- Trying %lu baud ---\r\n", bauds[i]);

        /* Power off for 2s to ensure a clean reset */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(2000);

        /* Configure baud rate and arm RX before power-on */
        optode_huart->Init.BaudRate = bauds[i];
        if (HAL_UART_Init(optode_huart) != HAL_OK) {
            shell_printf("[optode] UART init failed @ %lu\r\n", bauds[i]);
            continue;
        }
        optode_reset_uart();
        if (!optode_arm_rx(rx_buf, sizeof(rx_buf) - 1)) {
            shell_printf("[optode] arm_rx failed\r\n");
            continue;
        }

        /* Power on and listen */
        shell_printf("[optode] Power on, listening 8s...\r\n");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        uint16_t total = 0;
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < 8000 && total < sizeof(rx_buf) - 1) {
            if (rx_done) {
                if (rx_len > 0) {
                    total += rx_len;
                    shell_printf("[optode] chunk +%u (total %u) @ %lums\r\n",
                                 rx_len, total, HAL_GetTick() - t0);
                }
                uint16_t remaining = sizeof(rx_buf) - 1 - total;
                if (remaining == 0)
                    break;
                if (!optode_arm_rx(rx_buf + total, remaining))
                    break;
            }
        }
        HAL_UART_AbortReceive(optode_huart);

        if (total > 0 && optode_huart->ErrorCode == HAL_UART_ERROR_NONE) {
            rx_buf[total] = '\0';
            shell_printf("[optode] Match @ %lu baud! %u bytes:\r\n%s\r\n",
                         bauds[i], total, (char *)rx_buf);
            /* Restore 9600 if we matched at a different rate */
            if (bauds[i] != 9600) {
                shell_printf("[optode] NOTE: Use 'Set Baudrate(9600)' + "
                             "'Save' + 'Reset' to fix.\r\n");
            }
            return;
        }

        shell_printf("[optode] %lu baud: %s (err=0x%02lX)\r\n", bauds[i],
                     total == 0 ? "no data" : "framing errors",
                     optode_huart->ErrorCode);
    }

    /* Restore 9600 baud */
    optode_huart->Init.BaudRate = 9600;
    HAL_UART_Init(optode_huart);

    shell_printf("\r\n[optode] No match at any baud rate.\r\n");
}

