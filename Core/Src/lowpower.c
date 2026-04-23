/**
  ******************************************************************************
  * @file    lowpower.c
  * @brief   Low-power peripheral shutdown and restore helpers
  ******************************************************************************
  */

#include "lowpower.h"

#include "ctd.h"
#include "filesystem.h"
#include "main.h"
#include "optode.h"
#include "shell.h"
#include "wetlab.h"
#include "wifi.h"

#include <string.h>

/* External peripheral handles declared in main.c ---------------------------*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

typedef struct {
    bool pending;
    bool prepared;
    bool filesystem_was_mounted;
    bool sd_power_was_enabled;
    bool wifi_was_enabled;
} lowpower_state_t;

static lowpower_state_t g_lowpower_state;

/* Private helpers ----------------------------------------------------------*/

static void lowpower_config_output(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(port, pins, state);

    gpio.Pin = pins;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
}

static void lowpower_config_analog(GPIO_TypeDef *port, uint16_t pins, uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pins;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = pull;
    HAL_GPIO_Init(port, &gpio);
}

static void lowpower_restore_sd_cs_pin(void)
{
    lowpower_config_output(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

static void lowpower_restore_rtc_ce_pin(void)
{
    lowpower_config_output(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

/* Individual peripheral / pin-group helpers -------------------------------*/

void lowpower_shell_uart_down(void)
{
    HAL_UART_AbortReceive_IT(&huart1);
    HAL_UART_Abort(&huart1);
    HAL_UART_DeInit(&huart1);

    lowpower_config_analog(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PULLDOWN);
    lowpower_config_analog(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_NOPULL);
}

void lowpower_shell_uart_up(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    shell_resume_rx();
}

void lowpower_optode_uart_down(void)
{
    HAL_UART_Abort(&huart2);
    HAL_UART_DeInit(&huart2);

    lowpower_config_output(PB1_USART2_EN_GPIO_Port, PB1_USART2_EN_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PULLDOWN);
}

void lowpower_optode_uart_up(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    lowpower_config_output(PB1_USART2_EN_GPIO_Port, PB1_USART2_EN_Pin,
                           GPIO_PIN_SET);
    optode_init(&huart2);
}

void lowpower_ctd_uart_down(void)
{
    HAL_UART_Abort(&huart3);
    HAL_UART_DeInit(&huart3);

    lowpower_config_output(PB0_USART3_EN_GPIO_Port, PB0_USART3_EN_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PULLDOWN);
}

void lowpower_ctd_uart_up(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }

    lowpower_config_output(PB0_USART3_EN_GPIO_Port, PB0_USART3_EN_Pin, GPIO_PIN_SET);
    ctd_init(&huart3);
}

void lowpower_wifi_uart_down(void)
{
    wifi_down();
    HAL_UART_Abort(&huart4);
    HAL_UART_DeInit(&huart4);

    lowpower_config_output(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    lowpower_config_output(PB9_TRUCK_INT_OUT_GPIO_Port, PB9_TRUCK_INT_OUT_Pin, GPIO_PIN_SET);
    lowpower_config_analog(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PULLDOWN);
}

void lowpower_wifi_uart_up(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 9600;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }

    lowpower_config_output(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    lowpower_config_output(PB9_TRUCK_INT_OUT_GPIO_Port, PB9_TRUCK_INT_OUT_Pin, GPIO_PIN_SET);
}

void lowpower_wetlab_uart_down(void)
{
    HAL_UART_Abort(&huart5);
    HAL_UART_DeInit(&huart5);

    lowpower_config_output(GPIOB, PB4_AUX_SEL_A0_Pin | PB5_AUX_SEL_A1_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOC, GPIO_PIN_12, GPIO_PULLDOWN);
    lowpower_config_analog(GPIOD, GPIO_PIN_2, GPIO_PULLDOWN);
}

void lowpower_wetlab_uart_up(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 19200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
    }

    lowpower_config_output(GPIOB, PB4_AUX_SEL_A0_Pin | PB5_AUX_SEL_A1_Pin, GPIO_PIN_RESET);
    wetlab_init(&huart5);
}

void lowpower_sd_spi_down(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_SPI_DeInit(&hspi1);

    lowpower_config_output(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOA, SPI1_CS_Pin | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_NOPULL);
}

void lowpower_sd_spi_up(void)
{
    lowpower_config_output(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    lowpower_restore_sd_cs_pin();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

void lowpower_rtc_spi_down(void)
{
    lowpower_restore_rtc_ce_pin();
    HAL_SPI_DeInit(&hspi2);

    lowpower_restore_rtc_ce_pin();
    lowpower_config_analog(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_NOPULL);
}

void lowpower_rtc_spi_up(void)
{
    lowpower_restore_rtc_ce_pin();

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }

    lowpower_restore_rtc_ce_pin();
}

/* Aggregated low-power flow ------------------------------------------------*/

void lowpower_init(void)
{
    memset(&g_lowpower_state, 0, sizeof(g_lowpower_state));
}

bool lowpower_request_on(void)
{
    if (g_lowpower_state.pending) {
        return false;
    }

    g_lowpower_state.pending = true;
    return true;
}

bool lowpower_is_pending(void)
{
    return g_lowpower_state.pending;
}

void lowpower_prepare_for_sleep(void)
{
    if (g_lowpower_state.prepared) {
        return;
    }

    g_lowpower_state.filesystem_was_mounted = filesystem_is_mounted();
    g_lowpower_state.sd_power_was_enabled = (HAL_GPIO_ReadPin(SD_PWR_GPIO_Port, SD_PWR_Pin) == GPIO_PIN_SET);
    g_lowpower_state.wifi_was_enabled = (wifi_get_state() != WIFI_STATE_OFF);

    if (g_lowpower_state.filesystem_was_mounted) {
        (void)filesystem_unmount();
    }

    /* Keep PB8 dock sense and PB10 RTC interrupt active as wake sources. */
    lowpower_config_output(CLK_OE_GPIO_Port, CLK_OE_Pin, GPIO_PIN_RESET);
    lowpower_rtc_spi_down();
    lowpower_sd_spi_down();
    lowpower_wetlab_uart_down();
    lowpower_optode_uart_down();
    lowpower_ctd_uart_down();
    lowpower_wifi_uart_down();
    lowpower_shell_uart_down();

    g_lowpower_state.prepared = true;
}

void lowpower_restore_from_sleep(void)
{
    if (!g_lowpower_state.prepared) {
        return;
    }

    lowpower_shell_uart_up();
    lowpower_wifi_uart_up();
    lowpower_ctd_uart_up();
    lowpower_optode_uart_up();
    lowpower_wetlab_uart_up();

    if (g_lowpower_state.sd_power_was_enabled) {
        lowpower_sd_spi_up();
    }

    lowpower_rtc_spi_up();
    lowpower_config_output(CLK_OE_GPIO_Port, CLK_OE_Pin, GPIO_PIN_RESET);

    if (g_lowpower_state.filesystem_was_mounted && g_lowpower_state.sd_power_was_enabled) {
        FS_Result_t fs_status = filesystem_mount();
        if (fs_status != FS_OK && fs_status != FS_ALREADY_MOUNTED) {
            shell_printf("[lowpower] SD remount failed (err=%d)\r\n", fs_status);
        }
    }

    if (g_lowpower_state.wifi_was_enabled) {
        wifi_init(&huart4);
    }

    g_lowpower_state.prepared = false;
}

void lowpower_service(void)
{
    if (!g_lowpower_state.pending) {
        return;
    }

    g_lowpower_state.pending = false;

    shell_print("[lowpower] Preparing peripherals for sleep...\r\n");
    lowpower_prepare_for_sleep();

    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();

    lowpower_restore_from_sleep();

    shell_print("[lowpower] Woke from sleep.\r\n");
    shell_print(SHELL_PROMPT);
}
