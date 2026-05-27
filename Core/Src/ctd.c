/**
  ******************************************************************************
  * @file    ctd.c
  * @brief   Driver for the CTD sensor
  * @note    This driver uses the STM32 HAL library for UART communication.
  *          USART3 with DMA1 Ch2 (TX) and DMA1 Ch3 (RX).
  ******************************************************************************
  */

#include "ctd.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *ctd_huart;
static volatile bool tx_done = false;
static volatile bool rx_done = false;
static volatile uint16_t rx_len = 0;
static uint8_t rx_buf[CTD_RX_BUF_SIZE];

/* Callback notify functions (called from centralized HAL callbacks) ----------*/

void ctd_notify_tx_cplt(void)
{
    tx_done = true;
}

void ctd_notify_rx_event(uint16_t size)
{
    rx_len = size;
    rx_done = true;
}

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the CTD driver
  * @param  huart: UART handle for communication with the CTD sensor
  * @retval None
  */
void ctd_init(UART_HandleTypeDef *huart)
{
    ctd_huart = huart;
}

/** @brief  Reset the UART interface for the CTD driver
  * @param  None
  * @retval None
  */
static void ctd_reset_uart(void)
{
    HAL_UART_Abort(ctd_huart);
    __HAL_UART_CLEAR_OREFLAG(ctd_huart);
    __HAL_UART_CLEAR_NEFLAG(ctd_huart);
    __HAL_UART_CLEAR_FEFLAG(ctd_huart);
}

/** @brief  Send the "ts" command to the CTD sensor and parse the response
  * @param  out: Pointer to a ctd_data_t structure to receive the parsed data
  * @retval true if successful, false on error (e.g. timeout, parse failure)
  */
bool ctd_wakeup(void)
{
    /*
     * The first CR wakes the CTD hardware from quiescent mode, but the byte
     * itself is often lost (receiver wasn't active).  Retry until the device
     * actually responds, confirming it is awake and ready for commands.
     */
    for (int attempt = 0; attempt < CTD_WAKEUP_RETRIES; attempt++) {
        ctd_reset_uart();

        rx_done = false;
        rx_len = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(ctd_huart, rx_buf, sizeof(rx_buf) - 1);
        __HAL_DMA_DISABLE_IT(ctd_huart->hdmarx, DMA_IT_HT);

        tx_done = false;
        if (HAL_UART_Transmit_DMA(ctd_huart, (uint8_t *)"\r\n", 2) != HAL_OK) {
            HAL_UART_AbortReceive(ctd_huart);
            continue;
        }

        uint32_t t0 = HAL_GetTick();
        while (!tx_done) {
            if ((HAL_GetTick() - t0) > 1000) {
                HAL_UART_Abort(ctd_huart);
                break;
            }
        }

        /* guard in case tx didn't complete */
        if (!tx_done) {
            continue;
        }

        /* Wait up to 50 ms for the CTD to respond */
        t0 = HAL_GetTick();
        while (!rx_done) {
            if ((HAL_GetTick() - t0) > CTD_WAKEUP_TIMEOUT_MS)
                break;
        }

        HAL_UART_AbortReceive(ctd_huart);

        if (rx_done)
            return true;   /* CTD answered — it is awake */
    }

    return false;
}

/** @brief  Arm the RX DMA for the CTD driver
  * @param  buf: Buffer to receive data into
  * @param  len: Length of the buffer
  * @retval true if successful, false on error
  */
static bool ctd_arm_rx(uint8_t *buf, uint16_t len)
{
    rx_done = false;
    rx_len = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(ctd_huart, buf, len) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(ctd_huart->hdmarx, DMA_IT_HT);
    return true;
}

static bool ctd_parse_response(char *buf, ctd_data_t *out)
{
    /* Parse: find the data line, first line containing a comma. */
    char *line = strtok(buf, "\r\n");
    while (line) {
        if (strchr(line, ',')) {
            char *end;
            out->pressure = strtof(line, &end);
            if (*end != ',') {
                line = strtok(NULL, "\r\n");
                continue;
            }
            out->temperature = strtof(end + 1, &end);
            if (*end != ',') {
                line = strtok(NULL, "\r\n");
                continue;
            }
            out->conductivity = strtof(end + 1, &end);
            return true;
        }
        line = strtok(NULL, "\r\n");
    }
    return false;
}

/** @brief  Send the "ts" command to the CTD sensor and parse the response
  * @param  out: Pointer to a ctd_data_t structure to receive the parsed data
  * @retval true if successful, false on error (e.g. timeout, parse failure)
  */
bool ctd_ts(ctd_data_t *out)
{
    uint16_t total = ctd_sample_raw(NULL, 0);

    if (total == 0)
        return false;

    return ctd_parse_response((char *)rx_buf, out);
}

uint16_t ctd_sample_raw(uint8_t *out, uint16_t max_len)
{
    const char *cmd = "ts\r";
    uint32_t t0;
    uint16_t total = 0;

    if (!ctd_wakeup())
        return 0;

    /* Arm RX DMA before TX so we don't miss the response */
    if (!ctd_arm_rx(rx_buf, sizeof(rx_buf) - 1))
        return 0;

    /* TX command via DMA */
    tx_done = false;
    if (HAL_UART_Transmit_DMA(ctd_huart, (uint8_t *)cmd, strlen(cmd)) != HAL_OK) {
        HAL_UART_AbortReceive(ctd_huart);
        return 0;
    }
    t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_Abort(ctd_huart);
            return 0;
        }
    }

    /*
     * Collect response chunks.  The CTD echoes the command immediately, then
     * runs its pump (~4 s) before sending the measurement line followed by the
     * "S>" prompt.  We detect the comma in the measurement data, then break as
     * soon as the prompt '>' arrives.  10-s overall safety timeout.
     */
    t0 = HAL_GetTick();
    bool got_measurement = false;
    while ((HAL_GetTick() - t0) < 10000 && total < sizeof(rx_buf) - 1) {
        if (rx_done) {
            if (rx_len > 0) {
                total += rx_len;
                if (!got_measurement &&
                    memchr(rx_buf + total - rx_len, ',', rx_len))
                    got_measurement = true;
                if (got_measurement &&
                    memchr(rx_buf + total - rx_len, '>', rx_len))
                    break;
            }
            uint16_t remaining = sizeof(rx_buf) - 1 - total;
            if (remaining == 0)
                break;
            if (!ctd_arm_rx(rx_buf + total, remaining))
                break;
        }
    }
    HAL_UART_AbortReceive(ctd_huart);

    if (total == 0)
        return 0;

    rx_buf[total] = '\0';

    if (out != NULL && max_len > 0) {
        uint16_t copy_len = total;
        if (copy_len >= max_len)
            copy_len = max_len - 1;
        memcpy(out, rx_buf, copy_len);
        out[copy_len] = '\0';
    }

    return total;
}

bool ctd_fire(void)
{
    const char *cmd = "ts\r";
    const uint16_t buf_size = sizeof(rx_buf) - 1;

    ctd_reset_uart();

    /* Start continuous RX DMA — poll NDTR, no callbacks */
    if (HAL_UART_Receive_DMA(ctd_huart, rx_buf, buf_size) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(ctd_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(ctd_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(ctd_huart->Instance->CR3, USART_CR3_EIE);

    /* TX command via DMA */
    tx_done = false;
    if (HAL_UART_Transmit_DMA(ctd_huart, (uint8_t *)cmd, strlen(cmd)) != HAL_OK) {
        HAL_UART_AbortReceive(ctd_huart);
        return false;
    }
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_Abort(ctd_huart);
            return false;
        }
    }
    return true;
}

bool ctd_collect(ctd_data_t *out)
{
    const uint16_t buf_size = sizeof(rx_buf) - 1;
    uint16_t total = buf_size - __HAL_DMA_GET_COUNTER(ctd_huart->hdmarx);
    HAL_UART_AbortReceive(ctd_huart);

    if (total == 0)
        return false;

    rx_buf[total] = '\0';

    return ctd_parse_response((char *)rx_buf, out);
}
