/**
 * @file lowpower.c
 * @brief SD and external RTC SPI power helpers for the shell/LI-COR branch.
 * This branch stays awake; there is no profile or WiFi power service.
 */
#include "lowpower.h"
#include "config.h"
#include "filesystem.h"
#include "main.h"
#include "user_diskio.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

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

/**
 * @brief Configure GPIO pins as analog inputs for low-leakage idle state.
 * @param port GPIO port containing the pins.
 * @param pins GPIO pin mask to configure.
 * @param pull GPIO pull configuration to apply.
 */
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    lowpower_config_output(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Restore the external RTC chip-enable pin to its idle state.
 */
static void lowpower_restore_rtc_ce_pin(void)
{
    lowpower_config_output(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

void lowpower_sd_spi_down(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    filesystem_force_reset();
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
    HAL_SPI_Abort(&hspi1);
    HAL_SPI_DeInit(&hspi1);
    USER_diskio_reset();

    lowpower_config_output(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOA, SPI1_CS_Pin | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_NOPULL);
    g_app.sd_status = PERIPH_OFF;
}

/**
 * @brief Power up and reinitialize the SD card SPI path.
 */
void lowpower_sd_spi_up(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    USER_diskio_reset();
    lowpower_restore_sd_cs_pin();

    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

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

    lowpower_restore_sd_cs_pin();
    lowpower_config_output(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

static void lowpower_rtc_spi_down(void)
{
    lowpower_restore_rtc_ce_pin();
    HAL_SPI_DeInit(&hspi2);

    lowpower_restore_rtc_ce_pin();
    lowpower_config_analog(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_NOPULL);
}

/**
 * @brief Restore and reinitialize the external RTC SPI path.
 */
static void lowpower_rtc_spi_up(void)
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

/* Each RTC shell command temporarily acquires SPI2. */
bool lowpower_rtc_begin(void)
{
    lowpower_rtc_spi_up();
    return true;
}

void lowpower_rtc_end(bool release_spi)
{
    if (release_spi) {
        lowpower_rtc_spi_down();
    }
}
