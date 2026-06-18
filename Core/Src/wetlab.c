/**
  ******************************************************************************
  * @file    wetlab.c
  * @brief   Driver for the WetLab ECO sensor
  * @note    UART5 with DMA2 Ch2 (RX only).
  *          Sensor auto-transmits at 19200 baud on power-on (AutoRun).
  *          Power is toggled via PB4.
  ******************************************************************************
  */

#include "wetlab.h"
#include "config.h"
#include "shell.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define WETLAB_BUF_SIZE   4096

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *wetlab_huart;
static uint8_t rx_buf[WETLAB_BUF_SIZE];


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

    if (HAL_UART_Receive_DMA(wetlab_huart, rx_buf, WETLAB_BUF_SIZE - 1) != HAL_OK) {
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
    uint16_t total = (WETLAB_BUF_SIZE - 1) - __HAL_DMA_GET_COUNTER(wetlab_huart->hdmarx);
    HAL_UART_AbortReceive(wetlab_huart);

    if (total < WETLAB_BUF_SIZE) {
        rx_buf[total] = '\0';
    }

    /* Remove any NULs from power-on transient by shifting the remaining data left */
    uint16_t dst = 0;
    for (uint16_t src = 0; src < total; src++) {
        if (rx_buf[src] != '\0')
            rx_buf[dst++] = rx_buf[src];
    }

    total = dst;
    rx_buf[total] = '\0';

    return total;
}

/**
 * @brief  Parse a data row into wetlab_data_t.
 * @param  line  Tab-separated: MM/DD/YY HH:MM:SS lambda1 sig1 lambda2 sig2 lambda3 sig3 therm
 * @return true on success.
 */
static bool wetlab_parse(const char *line, wetlab_data_t *out)
{
    unsigned m, d, y, hh, mm, ss;
    unsigned ch1_lambda, ch1_signal, ch2_lambda, ch2_signal, ch3_lambda, ch3_signal, therm;
    if (sscanf(line, "%u/%u/%u %u:%u:%u %u %u %u %u %u %u %u",
               &m, &d, &y, &hh, &mm, &ss,
               &ch1_lambda, &ch1_signal, &ch2_lambda, &ch2_signal, &ch3_lambda, &ch3_signal, &therm) != 13) {
        return false;
    }
    /* Validate signal counts are within classic mode range (0–4130) */
    if (ch1_signal > 4130 || ch2_signal > 4130 || ch3_signal > 4130)
        return false;

    out->ch1_lambda   = (uint16_t)ch1_lambda;
    out->ch1_signal   = (uint16_t)ch1_signal;
    out->ch2_lambda   = (uint16_t)ch2_lambda;
    out->ch2_signal   = (uint16_t)ch2_signal;
    out->ch3_lambda   = (uint16_t)ch3_lambda;
    out->ch3_signal   = (uint16_t)ch3_signal;
    out->thermistor   = therm;
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
        while (*p == '\r' || *p == '\n') {
            p++;
        }

        if (!*p) {
            break;
        }

        char *eol = p;

        while (*eol && *eol != '\r' && *eol != '\n') {
            eol++;
        }

        if (!*eol) {
            break;  /* incomplete line at end of buffer — skip */
        }

        *eol = '\0';

        if (*p >= '0' && *p <= '9') {
            last = p;
        }

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

    HAL_Delay(WETLAB_SAMPLE_WAIT_MS);

    uint16_t total = wetlab_stop_rx();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

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
