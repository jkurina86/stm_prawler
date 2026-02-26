/**
  ******************************************************************************
  * @file    wetlab.c
  * @brief   Driver for the WetLab ECO Triplet (FLBBCDPML) sensor
  * @note    UART5 with DMA2 Ch1 (TX) and DMA2 Ch2 (RX).
  *          Sensor auto-transmits at 19200 baud on power-on (AutoRun).
  *          Power is toggled via PB4.
  ******************************************************************************
  */

#include "wetlab.h"
#include "shell.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
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
 * @brief  Arm polling DMA RX on UART5.
 * @return true on success.
 */
static bool wetlab_arm_rx(void)
{
    wetlab_reset_uart();
    __disable_irq();
    if (HAL_UART_Receive_DMA(wetlab_huart, rx_buf,
                              WETLAB_BUF_SIZE - 1) != HAL_OK) {
        __enable_irq();
        return false;
    }
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(wetlab_huart->hdmarx, DMA_IT_TC);
    CLEAR_BIT(wetlab_huart->Instance->CR3, USART_CR3_EIE);
    __enable_irq();
    return true;
}

/**
 * @brief  Stop DMA RX, null-terminate buffer, return byte count.
 */
static uint16_t wetlab_stop_rx(void)
{
    uint16_t total = (WETLAB_BUF_SIZE - 1)
                   - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
    HAL_UART_AbortReceive(wetlab_huart);
    if (total < WETLAB_BUF_SIZE)
        rx_buf[total] = '\0';
    /* Replace any embedded NULs (from power-on transient) with spaces
     * so string functions can scan the full buffer */
    for (uint16_t i = 0; i < total; i++) {
        if (rx_buf[i] == '\0')
            rx_buf[i] = ' ';
    }
    return total;
}

/**
 * @brief  Send a buffer via DMA TX and spin-wait for completion.
 * @return true if TX completed, false on timeout.
 */
static bool wetlab_tx(const uint8_t *data, uint16_t len)
{
    tx_done = false;
    if (HAL_UART_Transmit_DMA(wetlab_huart, data, len) != HAL_OK)
        return false;
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_AbortTransmit(wetlab_huart);
            return false;
        }
    }
    return true;
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

/**
 * @brief  Find the last complete data row in the buffer.
 * @return Pointer to the start of the line, or NULL.
 */
static char *wetlab_find_last_row(void)
{
    char *last = NULL;
    char *p = (char *)rx_buf;
    while (*p) {
        while (*p == '\r' || *p == '\n')
            p++;
        if (!*p)
            break;
        char *eol = p;
        while (*eol && *eol != '\r' && *eol != '\n')
            eol++;
        if (!*eol)
            break;  /* incomplete line at end of buffer — skip */
        *eol = '\0';
        if (*p >= '0' && *p <= '9')
            last = p;
        p = eol + 1;
    }
    return last;
}

/* Public functions ----------------------------------------------------------*/

void wetlab_init(UART_HandleTypeDef *huart)
{
    wetlab_huart = huart;
}

bool wetlab_sample(wetlab_data_t *out)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(1500);

    if (!wetlab_arm_rx()) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        return false;
    }

    HAL_Delay(900);

    uint16_t total = wetlab_stop_rx();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    shell_printf("[wetlab] DMA got %u bytes: \"%s\"\r\n", total, (char *)rx_buf);

    if (total == 0)
        return false;

    char *line = wetlab_find_last_row();
    if (!line)
        return false;

    return wetlab_parse(line, out);
}

void wetlab_raw(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);

    if (!wetlab_arm_rx()) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        shell_printf("[wetlab] Failed to start DMA\r\n");
        return;
    }

    HAL_Delay(4000);

    uint16_t total = wetlab_stop_rx();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    if (total == 0) {
        shell_printf("[wetlab] No data received\r\n");
        return;
    }

    shell_printf("[wetlab] %u bytes:\r\n", total);
    shell_print((char *)rx_buf);
    shell_print("\r\n");
}

void wetlab_cmd(const char *cmd)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100);

    if (!wetlab_arm_rx()) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        shell_printf("[wetlab] Failed to start DMA\r\n");
        return;
    }

    /* Wait for sensor to boot and start auto-run, then stop it */
    HAL_Delay(2000);
    if (!wetlab_tx((const uint8_t *)"!!!!!", 5)) {
        HAL_UART_Abort(wetlab_huart);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        shell_printf("[wetlab] TX failed\r\n");
        return;
    }

    /* Let it settle into standby, discard auto-run data */
    HAL_Delay(2000);
    HAL_UART_AbortReceive(wetlab_huart);
    if (!wetlab_arm_rx()) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        shell_printf("[wetlab] Failed to re-arm DMA\r\n");
        return;
    }

    /* Send command */
    if (!wetlab_tx((const uint8_t *)cmd, strlen(cmd))) {
        HAL_UART_Abort(wetlab_huart);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        shell_printf("[wetlab] TX failed\r\n");
        return;
    }

    HAL_Delay(5000);

    uint16_t total = wetlab_stop_rx();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    if (total == 0) {
        shell_printf("[wetlab] No data received\r\n");
        return;
    }

    shell_printf("[wetlab] %u bytes:\r\n", total);
    shell_print((char *)rx_buf);
    shell_print("\r\n");
}

/* Split-phase API for simultaneous sampling --------------------------------*/

bool wetlab_fire(void)
{
    if (!wetlab_arm_rx())
        return false;
    return true;
}

bool wetlab_collect(wetlab_data_t *out)
{
    uint16_t total = wetlab_stop_rx();

    if (total == 0)
        return false;

    char *line = wetlab_find_last_row();
    if (!line)
        return false;

    return wetlab_parse(line, out);
}
