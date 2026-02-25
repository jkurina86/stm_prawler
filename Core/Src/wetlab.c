/**
  ******************************************************************************
  * @file    wetlab.c
  * @brief   Driver for the WetLab sensor
  * @note    UART5 with DMA2 Ch1 (TX) and DMA2 Ch2 (RX).
  *          The sensor auto-transmits measurements after power-on.
  *          Power is toggled via PB4 (reset, wait, set).
  ******************************************************************************
  */

#include "wetlab.h"
#include "shell.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define WETLAB_BAUD       19200
#define WETLAB_BUF_SIZE   1024

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *wetlab_huart;
static volatile bool tx_done = false;
static uint8_t rx_buf[WETLAB_BUF_SIZE];

/* Callback notify functions (called from centralized HAL callbacks) ----------*/

void wetlab_notify_tx_cplt(void)
{
    tx_done = true;
}

void wetlab_notify_rx_event(uint16_t size)
{
    /* Not used — polling DMA counter instead of callbacks */
    (void)size;
}

/* Private functions ---------------------------------------------------------*/

static void wetlab_reset_uart(void)
{
    HAL_UART_Abort(wetlab_huart);
    __HAL_UART_CLEAR_OREFLAG(wetlab_huart);
    __HAL_UART_CLEAR_NEFLAG(wetlab_huart);
    __HAL_UART_CLEAR_FEFLAG(wetlab_huart);
}

/**
 * @brief  Parse a data row into wetlab_data_t.
 * @param  line  Tab-separated: MM/DD/YY HH:MM:SS λ1 sig1 λ2 sig2 λ3 sig3 therm
 * @return true on success.
 */
static bool wetlab_parse(const char *line, wetlab_data_t *out)
{
    unsigned m, d, y, hh, mm, ss;
    unsigned cl, cs, nl, ns, dl, ds, th;
    if (sscanf(line, "%u/%u/%u %u:%u:%u %u %u %u %u %u %u %u",
               &m, &d, &y, &hh, &mm, &ss,
               &cl, &cs, &nl, &ns, &dl, &ds, &th) != 13)
        return false;
    out->chl_lambda   = (uint16_t)cl;
    out->chl_signal   = (uint16_t)cs;
    out->ntu_lambda   = (uint16_t)nl;
    out->ntu_signal   = (uint16_t)ns;
    out->cdom_lambda  = (uint16_t)dl;
    out->cdom_signal  = (uint16_t)ds;
    out->thermistor   = (uint16_t)th;
    return true;
}

/* Public functions ----------------------------------------------------------*/

void wetlab_init(UART_HandleTypeDef *huart)
{
    wetlab_huart = huart;
}

bool wetlab_sample(wetlab_data_t *out)
{
    const uint16_t buf_size = WETLAB_BUF_SIZE - 1;

    /* Switch UART5 to sensor baud rate */
    wetlab_huart->Init.BaudRate = WETLAB_BAUD;
    if (HAL_UART_Init(wetlab_huart) != HAL_OK) {
        shell_printf("[wetlab] UART init failed\r\n");
        return false;
    }

    /* Power off the sensor */
    shell_printf("[wetlab] Power-cycling sensor...\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /* Power on, let the line settle before starting DMA */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);

    /* Clear errors from power-on transient, then start one continuous
     * DMA transfer for the entire buffer.  Disable all DMA and UART error
     * interrupts so nothing can abort the transfer — we poll NDTR instead. */
    wetlab_reset_uart();
    if (HAL_UART_Receive_DMA(wetlab_huart, rx_buf, buf_size) != HAL_OK) {
        shell_printf("[wetlab] Failed to start DMA\r\n");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        return false;
    }
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(wetlab_huart->Instance->CR3, USART_CR3_EIE);

    /* Poll DMA counter for up to 2 seconds. */
    uint32_t t0 = HAL_GetTick();
    uint16_t prev_received = 0;

    while ((HAL_GetTick() - t0) < 2500) {
        uint16_t received = buf_size - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
        if (received != prev_received) {
            prev_received = received;
        }
    }

    uint16_t total = buf_size - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
    HAL_UART_AbortReceive(wetlab_huart);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    if (total == 0) {
        shell_printf("[wetlab] No data received\r\n");
        return false;
    }

    rx_buf[total] = '\0';

    /* Find the 3rd data row (skip "mvs 1" header and first 2 data lines) */
    int row = 0;
    char *p = (char *)rx_buf;
    char *target = NULL;
    while (*p) {
        /* Skip leading whitespace/newlines */
        while (*p == '\r' || *p == '\n')
            p++;
        if (!*p)
            break;
        /* Find end of this line */
        char *eol = p;
        while (*eol && *eol != '\r' && *eol != '\n')
            eol++;
        /* Skip "mvs" lines */
        if (p[0] != 'm' || p[1] != 'v' || p[2] != 's') {
            row++;
            if (row == 3) {
                target = p;
                *eol = '\0';
                break;
            }
        }
        p = eol;
    }

    if (!target) {
        shell_printf("[wetlab] Less than 3 data rows\r\n");
        return false;
    }

    if (!wetlab_parse(target, out)) {
        shell_printf("[wetlab] Parse failed: %s\r\n", target);
        return false;
    }

    return true;
}

void wetlab_raw(void)
{
    const uint16_t buf_size = WETLAB_BUF_SIZE - 1;

    wetlab_huart->Init.BaudRate = WETLAB_BAUD;
    if (HAL_UART_Init(wetlab_huart) != HAL_OK) {
        shell_printf("[wetlab] UART init failed\r\n");
        return;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);

    wetlab_reset_uart();
    if (HAL_UART_Receive_DMA(wetlab_huart, rx_buf, buf_size) != HAL_OK) {
        shell_printf("[wetlab] Failed to start DMA\r\n");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        return;
    }
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(wetlab_huart->Instance->CR3, USART_CR3_EIE);

    HAL_Delay(4000);

    uint16_t total = buf_size - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
    HAL_UART_AbortReceive(wetlab_huart);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    if (total == 0) {
        shell_printf("[wetlab] No data received\r\n");
        return;
    }

    rx_buf[total] = '\0';
    shell_printf("[wetlab] %u bytes:\r\n", total);
    shell_print((char *)rx_buf);
    shell_print("\r\n");
}

void wetlab_cmd(const char *cmd)
{
    const uint16_t buf_size = WETLAB_BUF_SIZE - 1;

    wetlab_huart->Init.BaudRate = WETLAB_BAUD;
    if (HAL_UART_Init(wetlab_huart) != HAL_OK) {
        shell_printf("[wetlab] UART init failed\r\n");
        return;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);

    wetlab_reset_uart();
    if (HAL_UART_Receive_DMA(wetlab_huart, rx_buf, buf_size) != HAL_OK) {
        shell_printf("[wetlab] Failed to start RX DMA\r\n");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        return;
    }
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(wetlab_huart->Instance->CR3, USART_CR3_EIE);

    /* Wait for sensor to boot and start auto-run, then stop it */
    HAL_Delay(2000);

    tx_done = false;
    if (HAL_UART_Transmit_DMA(wetlab_huart,
                               (uint8_t *)"!!!!!", 5) != HAL_OK) {
        shell_printf("[wetlab] TX failed\r\n");
        HAL_UART_Abort(wetlab_huart);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        return;
    }
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_Abort(wetlab_huart);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            shell_printf("[wetlab] TX timeout\r\n");
            return;
        }
    }

    /* Let it settle into standby */
    HAL_Delay(2000);

    /* Send command */
    tx_done = false;
    if (HAL_UART_Transmit_DMA(wetlab_huart,
                               (uint8_t *)cmd, strlen(cmd)) != HAL_OK) {
        shell_printf("[wetlab] TX failed\r\n");
        HAL_UART_Abort(wetlab_huart);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        return;
    }
    t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_Abort(wetlab_huart);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            shell_printf("[wetlab] TX timeout\r\n");
            return;
        }
    }

    /* Listen for response */
    HAL_Delay(5000);

    uint16_t total = buf_size - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
    HAL_UART_AbortReceive(wetlab_huart);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    if (total == 0) {
        shell_printf("[wetlab] No data received\r\n");
        return;
    }

    rx_buf[total] = '\0';
    shell_printf("[wetlab] %u bytes:\r\n", total);
    shell_print((char *)rx_buf);
    shell_print("\r\n");
}

/* Split-phase API for simultaneous sampling --------------------------------*/

bool wetlab_fire(void)
{
    const uint16_t buf_size = WETLAB_BUF_SIZE - 1;

    /* Switch UART5 to sensor baud rate */
    wetlab_huart->Init.BaudRate = WETLAB_BAUD;
    if (HAL_UART_Init(wetlab_huart) != HAL_OK)
        return false;

    wetlab_reset_uart();

    /* Start continuous RX DMA — poll NDTR, no callbacks */
    if (HAL_UART_Receive_DMA(wetlab_huart, rx_buf, buf_size) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(wetlab_huart->Instance->CR3, USART_CR3_EIE);

    return true;
}

bool wetlab_collect(wetlab_data_t *out)
{
    const uint16_t buf_size = WETLAB_BUF_SIZE - 1;
    uint16_t total = buf_size - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
    HAL_UART_AbortReceive(wetlab_huart);

    if (total == 0)
        return false;

    rx_buf[total] = '\0';

    /* Find the 3rd data row (skip "mvs 1" header and first 2 data lines) */
    int row = 0;
    char *p = (char *)rx_buf;
    char *target = NULL;
    while (*p) {
        while (*p == '\r' || *p == '\n')
            p++;
        if (!*p)
            break;
        char *eol = p;
        while (*eol && *eol != '\r' && *eol != '\n')
            eol++;
        if (p[0] != 'm' || p[1] != 'v' || p[2] != 's') {
            row++;
            if (row == 3) {
                target = p;
                *eol = '\0';
                break;
            }
        }
        p = eol;
    }

    if (!target)
        return false;

    return wetlab_parse(target, out);
}
