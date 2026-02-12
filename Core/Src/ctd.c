/**
  ******************************************************************************
  * @file    ctd.c
  * @brief   Driver for the CTD sensor
  * @note    This driver uses the STM32 HAL library for UART communication.
  *          USART3 with DMA1 Ch2 (TX) and DMA1 Ch3 (RX).
  ******************************************************************************
  */

#include "ctd.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *ctd_huart;
static volatile bool tx_done = false;
static volatile bool rx_done = false;
static volatile uint16_t rx_len = 0;
static uint8_t rx_buf[1024];

/* HAL DMA callbacks ---------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == ctd_huart->Instance)
        tx_done = true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == ctd_huart->Instance) {
        rx_len = Size;
        rx_done = true;
    }
}

/* Public functions ----------------------------------------------------------*/

void ctd_init(UART_HandleTypeDef *huart)
{
    ctd_huart = huart;
}

static void ctd_reset_uart(void)
{
    HAL_UART_Abort(ctd_huart);
    __HAL_UART_CLEAR_OREFLAG(ctd_huart);
    __HAL_UART_CLEAR_NEFLAG(ctd_huart);
    __HAL_UART_CLEAR_FEFLAG(ctd_huart);
}

static bool ctd_wakeup(void)
{
    ctd_reset_uart();

    /* Arm RX to drain the wake prompt (e.g. "S>") */
    rx_done = false;
    rx_len = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(ctd_huart, rx_buf, sizeof(rx_buf) - 1);
    __HAL_DMA_DISABLE_IT(ctd_huart->hdmarx, DMA_IT_HT);

    tx_done = false;
    if (HAL_UART_Transmit_DMA(ctd_huart, (uint8_t *)"\r\r", 2) != HAL_OK)
        return false;
    uint32_t t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000)
            return false;
    }
    HAL_Delay(2000);

    /* Stop RX, discard whatever the CTD sent during wake */
    HAL_UART_AbortReceive(ctd_huart);
    return true;
}

static bool ctd_arm_rx(uint8_t *buf, uint16_t len)
{
    rx_done = false;
    rx_len = 0;
    if (HAL_UARTEx_ReceiveToIdle_DMA(ctd_huart, buf, len) != HAL_OK)
        return false;
    __HAL_DMA_DISABLE_IT(ctd_huart->hdmarx, DMA_IT_HT);
    return true;
}

bool ctd_ts(void (*print)(const char *))
{
    const char *cmd = "ts\r";
    uint32_t t0;
    uint16_t total = 0;

    if (!ctd_wakeup()) {
        print("CTD wakeup failed\r\n");
        return false;
    }

    /* Arm RX DMA before TX so we don't miss the response */
    if (!ctd_arm_rx(rx_buf, sizeof(rx_buf) - 1)) {
        print("CTD RX error\r\n");
        return false;
    }

    /* TX command via DMA */
    tx_done = false;
    if (HAL_UART_Transmit_DMA(ctd_huart, (uint8_t *)cmd, strlen(cmd)) != HAL_OK) {
        HAL_UART_AbortReceive(ctd_huart);
        print("CTD TX error\r\n");
        return false;
    }
    t0 = HAL_GetTick();
    while (!tx_done) {
        if ((HAL_GetTick() - t0) > 1000) {
            HAL_UART_Abort(ctd_huart);
            print("CTD TX timeout\r\n");
            return false;
        }
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 4000 && total < sizeof(rx_buf) - 1) {
        if (rx_done) {
            total += rx_len;
            uint16_t remaining = sizeof(rx_buf) - 1 - total;
            if (remaining == 0)
                break;
            if (!ctd_arm_rx(rx_buf + total, remaining))
                break;
        }
    }
    HAL_UART_AbortReceive(ctd_huart);

    if (total == 0) {
        print("CTD no response\r\n");
        return false;
    }

    rx_buf[total] = '\0';
    print((char *)rx_buf);
    print("\r\n");
    return true;
}
