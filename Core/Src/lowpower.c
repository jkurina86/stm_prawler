/**
  ******************************************************************************
  * @file    lowpower.c
  * @brief   Low-power peripheral shutdown and restore helpers
  ******************************************************************************
  */

#include "lowpower.h"

#include "ab-rtcmc-rtc.h"
#include "config.h"
#include "ctd.h"
#include "filesystem.h"
#include "main.h"
#include "optode.h"
#include "realtime_comm.h"
#include "shell.h"
#include "tasker.h"
#include "user_diskio.h"
#include "wetlab.h"
#include "wifi.h"

#include <string.h>

#define LOWPOWER_IDLE_TIMEOUT_SECONDS 60UL
#define LOWPOWER_IDLE_TIMEOUT_MS (LOWPOWER_IDLE_TIMEOUT_SECONDS * 1000UL)
#define LOWPOWER_STOP2_RETURN_TIMEOUT_MS 5000UL
#define LOWPOWER_RTC_WAKE_SECONDS 20UL

/* External peripheral handles declared in main.c ---------------------------*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern IWDG_HandleTypeDef hiwdg;

typedef struct {
    bool pending;
    bool prepared;
    bool filesystem_was_mounted;
    bool sd_power_was_enabled;
    bool profile_peripherals_up;
    bool profile_peripherals_were_up;
    bool quiet_sleep_entry;
    bool idle_timer_enabled;
    volatile bool idle_timer_active;
    volatile uint32_t idle_started_tick;
    volatile uint32_t idle_timeout_ms;
} lowpower_state_t;

static lowpower_state_t g_lowpower_state;

/* Private helpers ----------------------------------------------------------*/

/**
 * @brief Drive GPIO pins and configure them as low-speed push-pull outputs.
 * @param port GPIO port containing the pins.
 * @param pins GPIO pin mask to configure.
 * @param state Output state to drive before configuring the pins.
 */
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

/**
 * @brief Restore the SD card SPI chip-select pin to its idle state.
 */
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

/**
 * @brief Unmount the filesystem if FatFS is currently mounted.
 */
static void lowpower_unmount_filesystem(void)
{
    if (filesystem_is_mounted()) {
        (void)filesystem_unmount();
    }
}

/**
 * @brief Program the external RTC countdown timer as a one-shot wake source.
 * @param seconds Countdown interval in seconds.
 * @return RTC_OK on success, otherwise an RTC_Status_t error code.
 */
static RTC_Status_t lowpower_program_rtc_countdown(uint16_t seconds)
{
    RTC_Timer_t timer = {0};
    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_DisableInterrupt(RTC_INTERRUPT_MASK_ALL);

    if (status == RTC_OK) {
        status = RTC_EnableTimer(false);
    }
    if (status == RTC_OK) {
        status = RTC_ClearInterruptSources(RTC_INTERRUPT_MASK_ALL);
    }

    if (status == RTC_OK) {
        timer.timer_value = seconds;
        timer.division = RTC_TIMER_DIV_1HZ;
        timer.auto_reload = false;
        timer.enabled = false;
        status = RTC_SetTimer(&timer);
    }

    if (status == RTC_OK) {
        status = RTC_ClearTimerFlag();
    }
    if (status == RTC_OK) {
        status = RTC_EnableTimerInterrupt(true);
    }
    if (status == RTC_OK && RTC_IsInterruptAsserted()) {
        status = RTC_ERROR;
    }
    if (status == RTC_OK) {
        status = RTC_EnableTimer(true);
    }

    RTC_ClearPendingInterrupt();
    __HAL_GPIO_EXTI_CLEAR_IT(RTC_INT_PIN);
    lowpower_rtc_end(release_spi);
    return status;
}

/**
 * @brief Stop and clear the external RTC countdown timer wake source.
 * @return RTC_OK on success, otherwise an RTC_Status_t error code.
 */
static RTC_Status_t lowpower_stop_rtc_countdown(void)
{
    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_EnableTimerInterrupt(false);

    if (status == RTC_OK) {
        status = RTC_EnableTimer(false);
    }
    if (status == RTC_OK) {
        status = RTC_ClearTimerFlag();
    }

    RTC_ClearPendingInterrupt();
    __HAL_GPIO_EXTI_CLEAR_IT(RTC_INT_PIN);
    lowpower_rtc_end(release_spi);
    return status;
}

/**
 * @brief Clear peripheral NVIC pending bits that could immediately wake STOP2.
 */
static void lowpower_clear_stop2_pending_irqs(void)
{
    HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
    HAL_NVIC_ClearPendingIRQ(UART4_IRQn);
    HAL_NVIC_ClearPendingIRQ(UART5_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}

/**
 * @brief Clear the ARM event latch before entering WFE sleep.
 */
static void lowpower_clear_event_latch(void)
{
    __SEV();
    __WFE();
}

/**
 * @brief Enter STOP2 using WFE and return when an event wakes the CPU.
 */
static void lowpower_enter_stop2_wfe(void)
{
    MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP2);
    SET_BIT(SCB->SCR, (uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    __DSB();
    __WFE();
    CLEAR_BIT(SCB->SCR, (uint32_t)SCB_SCR_SLEEPDEEP_Msk);
}

/**
 * @brief Consume a pending PB8 record-trigger EXTI event.
 * @return true if a pending trigger was consumed, otherwise false.
 */
static bool lowpower_take_pending_record_trigger(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(RECORD_TRIGGER_Pin) == RESET) {
        return false;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(RECORD_TRIGGER_Pin);
    g_app.start_flag = true;
    return true;
}

/**
 * @brief Clear stale wake state and verify STOP2 entry is still safe.
 * @param rtc_wake Set true when the RTC wake line is already pending or asserted.
 * @return true if STOP2 can be entered, otherwise false.
 */
static bool lowpower_prepare_stop2_entry(bool *rtc_wake)
{
    if (lowpower_take_pending_record_trigger() || g_app.start_flag) {
        return false;
    }

    if (RTC_IsInterruptPending() || RTC_IsInterruptAsserted()) {
        *rtc_wake = true;
        return false;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(RECORD_TRIGGER_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(RTC_INT_PIN);
    lowpower_clear_stop2_pending_irqs();
    lowpower_clear_event_latch();
    __DSB();
    __ISB();

    if (lowpower_take_pending_record_trigger() || g_app.start_flag) {
        return false;
    }

    if (RTC_IsInterruptPending() || RTC_IsInterruptAsserted()) {
        *rtc_wake = true;
        return false;
    }

    return true;
}

/**
 * @brief Program the standard RTC wake timer used for STOP2 sleep windows.
 * @return RTC_OK on success, otherwise an RTC_Status_t error code.
 */
static RTC_Status_t lowpower_program_rtc_wake_timer(void)
{
    return lowpower_program_rtc_countdown((uint16_t)LOWPOWER_RTC_WAKE_SECONDS);
}

/**
 * @brief Start or restart the SysTick-based idle timer.
 * @param timeout_ms Idle timeout in milliseconds.
 */
static void lowpower_start_idle_timer(uint32_t timeout_ms)
{
    if (!g_lowpower_state.idle_timer_enabled) {
        g_lowpower_state.idle_timer_active = false;
        return;
    }

    g_lowpower_state.idle_started_tick = HAL_GetTick();
    g_lowpower_state.idle_timeout_ms = timeout_ms;
    g_lowpower_state.idle_timer_active = true;
}

/* Individual peripheral / pin-group helpers -------------------------------*/

/**
 * @brief Deinitialize the shell UART and park its pins for low power.
 */
void lowpower_shell_uart_down(void)
{
    HAL_UART_AbortReceive_IT(&huart1);
    HAL_UART_Abort(&huart1);
    HAL_UART_DeInit(&huart1);

    lowpower_config_analog(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PULLDOWN);
    lowpower_config_analog(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_NOPULL);
}

/**
 * @brief Restore the shell UART and resume interrupt-driven shell RX.
 */
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

/**
 * @brief Deinitialize the optode UART path and mark the optode off.
 */
void lowpower_optode_uart_down(void)
{
    HAL_UART_Abort(&huart2);
    HAL_UART_DeInit(&huart2);

    lowpower_config_output(PB1_USART2_EN_GPIO_Port, PB1_USART2_EN_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PULLDOWN);
    g_app.optode_status = PERIPH_OFF;
}

/**
 * @brief Restore the optode UART path and initialize the optode driver.
 */
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
    g_app.optode_status = PERIPH_READY;
}

/**
 * @brief Deinitialize the CTD UART path and mark the CTD off.
 */
void lowpower_ctd_uart_down(void)
{
    HAL_UART_Abort(&huart3);
    HAL_UART_DeInit(&huart3);

    lowpower_config_output(PB0_USART3_EN_GPIO_Port, PB0_USART3_EN_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PULLDOWN);
    g_app.ctd_status = PERIPH_OFF;
}

/**
 * @brief Restore the CTD UART path and initialize the CTD driver.
 */
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
    g_app.ctd_status = PERIPH_READY;
}

/**
 * @brief Power down the WiFi UART/module path and park its pins.
 */
void lowpower_wifi_uart_down(void)
{
    wifi_down();
    HAL_UART_Abort(&huart4);
    HAL_UART_DeInit(&huart4);

    lowpower_config_output(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    lowpower_config_output(PB9_TRUCK_INT_OUT_GPIO_Port, PB9_TRUCK_INT_OUT_Pin, GPIO_PIN_SET);
    lowpower_config_analog(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PULLDOWN);
}

/**
 * @brief Restore the WiFi UART pins and module control lines.
 */
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

/**
 * @brief Deinitialize the WetLab UART path and mark WetLab off.
 */
void lowpower_wetlab_uart_down(void)
{
    HAL_UART_Abort(&huart5);
    HAL_UART_DeInit(&huart5);

    lowpower_config_output(GPIOB, PB4_AUX_SEL_A0_Pin | PB5_AUX_SEL_A1_Pin, GPIO_PIN_RESET);
    lowpower_config_analog(GPIOC, GPIO_PIN_12, GPIO_PULLDOWN);
    lowpower_config_analog(GPIOD, GPIO_PIN_2, GPIO_PULLDOWN);
    g_app.wetlab_status = PERIPH_OFF;
}

/**
 * @brief Restore the WetLab UART path and initialize the WetLab driver.
 */
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
    g_app.wetlab_status = PERIPH_READY;
}

/**
 * @brief Power down the SD card SPI path and reset filesystem/disk state.
 */
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

/**
 * @brief Deinitialize the external RTC SPI path and park its pins.
 */
void lowpower_rtc_spi_down(void)
{
    lowpower_restore_rtc_ce_pin();
    HAL_SPI_DeInit(&hspi2);

    lowpower_restore_rtc_ce_pin();
    lowpower_config_analog(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_NOPULL);
}

/**
 * @brief Restore and reinitialize the external RTC SPI path.
 */
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

/**
 * @brief Initialize low-power module state.
 */
void lowpower_init(void)
{
    memset(&g_lowpower_state, 0, sizeof(g_lowpower_state));
    g_lowpower_state.idle_timer_enabled = true;
}

/**
 * @brief Put the application in idle mode and arm the idle sleep timer.
 */
void lowpower_enter_idle(void)
{
    g_app.mode = SYS_MODE_IDLE;
    lowpower_note_activity();
}

/**
 * @brief Record foreground activity and restart the normal idle timer.
 */
void lowpower_note_activity(void)
{
    lowpower_start_idle_timer(LOWPOWER_IDLE_TIMEOUT_MS);
}

/**
 * @brief Disable automatic idle sleep until the timer is explicitly restarted.
 */
void lowpower_stay_awake(void)
{
    g_lowpower_state.idle_timer_enabled = false;
    g_lowpower_state.idle_timer_active = false;
    g_lowpower_state.pending = false;
    g_lowpower_state.quiet_sleep_entry = false;
    g_lowpower_state.idle_timeout_ms = LOWPOWER_IDLE_TIMEOUT_MS;
}

/**
 * @brief Re-enable and restart the normal automatic idle sleep timer.
 */
void lowpower_restart_timer(void)
{
    g_lowpower_state.idle_timer_enabled = true;
    g_lowpower_state.pending = false;
    g_lowpower_state.quiet_sleep_entry = false;
    lowpower_start_idle_timer(LOWPOWER_IDLE_TIMEOUT_MS);
}

/**
 * @brief Report whether the automatic idle sleep timer is enabled.
 * @return true if the idle timer is enabled, otherwise false.
 */
bool lowpower_idle_timer_enabled(void)
{
    return g_lowpower_state.idle_timer_enabled;
}

/**
 * @brief Get elapsed time for the currently active idle timer.
 * @return Elapsed idle time in milliseconds, or 0 if the timer is inactive.
 */
uint32_t lowpower_idle_elapsed_ms(void)
{
    if (!g_lowpower_state.idle_timer_active) {
        return 0;
    }

    return HAL_GetTick() - g_lowpower_state.idle_started_tick;
}

/**
 * @brief Power down peripherals that should remain off during idle.
 */
void lowpower_idle_peripherals_down(void)
{
    lowpower_unmount_filesystem();
    lowpower_sd_spi_down();
    lowpower_rtc_spi_down();
    lowpower_wetlab_uart_down();
    lowpower_optode_uart_down();
    lowpower_ctd_uart_down();
    g_lowpower_state.profile_peripherals_up = false;
}

/**
 * @brief Bring up the sensor profile peripheral set and stop WiFi access.
 */
void lowpower_profile_peripherals_up(void)
{
    if (g_lowpower_state.profile_peripherals_up) {
        return;
    }

    lowpower_unmount_filesystem();
    lowpower_sd_spi_down();
    lowpower_wifi_stop();
    lowpower_rtc_spi_up();
    lowpower_ctd_uart_up();
    lowpower_optode_uart_up();
    lowpower_wetlab_uart_up();

    g_lowpower_state.profile_peripherals_up = true;
}

/**
 * @brief Power down the sensor profile peripheral set.
 */
void lowpower_profile_peripherals_down(void)
{
    lowpower_unmount_filesystem();
    lowpower_sd_spi_down();
    lowpower_wetlab_uart_down();
    lowpower_optode_uart_down();
    lowpower_ctd_uart_down();
    lowpower_rtc_spi_down();

    g_lowpower_state.profile_peripherals_up = false;
}

/**
 * @brief Report whether the sensor profile peripheral set is powered.
 * @return true if profile peripherals are up, otherwise false.
 */
bool lowpower_profile_peripherals_are_up(void)
{
    return g_lowpower_state.profile_peripherals_up;
}

/**
 * @brief Temporarily acquire external RTC SPI access outside profile ownership.
 * @return true if the caller should release SPI with lowpower_rtc_end(), false
 *         if profile power already owns SPI2.
 */
bool lowpower_rtc_begin(void)
{
    if (g_lowpower_state.profile_peripherals_up) {
        return false;
    }

    lowpower_rtc_spi_up();
    return true;
}

/**
 * @brief Release external RTC SPI access acquired by lowpower_rtc_begin().
 * @param release_spi true to power SPI2 back down, false to leave profile-owned SPI2 up.
 */
void lowpower_rtc_end(bool release_spi)
{
    if (release_spi) {
        lowpower_rtc_spi_down();
    }
}

/**
 * @brief Power up UART4 and initialize the WiFi module/service.
 */
void lowpower_wifi_start(void)
{
    lowpower_wifi_uart_up();
    wifi_init(&huart4);
}

/**
 * @brief Power up UART4 and restore WiFi from saved module settings.
 */
static void lowpower_wifi_resume(void)
{
    lowpower_wifi_uart_up();
    wifi_resume(&huart4);
}

/**
 * @brief Stop the WiFi module/service and power down UART4.
 */
void lowpower_wifi_stop(void)
{
    lowpower_wifi_uart_down();
}

/**
 * @brief Restore WiFi after an RTC wake and arm the short return-to-STOP2 timer.
 */
void lowpower_start_wifi_duty_cycle(void)
{
    g_app.mode = SYS_MODE_IDLE;
    lowpower_wifi_resume();
    lowpower_start_idle_timer(LOWPOWER_STOP2_RETURN_TIMEOUT_MS);
}

/**
 * @brief Request low-power sleep from main-loop context.
 * @return true if a new request was queued, false if one was already pending.
 */
bool lowpower_request_on(void)
{
    if (g_lowpower_state.pending) {
        return false;
    }

    g_lowpower_state.pending = true;
    g_lowpower_state.quiet_sleep_entry = false;
    return true;
}

/**
 * @brief Report whether a low-power sleep request is pending.
 * @return true if sleep service work is pending, otherwise false.
 */
bool lowpower_is_pending(void)
{
    return g_lowpower_state.pending;
}

/**
 * @brief Check whether automatic idle sleep is currently allowed.
 * @return true if the idle timer may request sleep, otherwise false.
 */
static bool lowpower_idle_timer_allowed(void)
{
    if (!g_lowpower_state.idle_timer_active ||
        g_lowpower_state.pending ||
        g_lowpower_state.prepared ||
        g_lowpower_state.profile_peripherals_up ||
        g_app.mode != SYS_MODE_IDLE ||
        g_app.start_flag ||
        tasker_pending_count() != 0) {
        return false;
    }

    if (realtime_comm_data_pending()) {
        return false;
    }

    return true;
}

/**
 * @brief Service the automatic idle timer and request sleep when it expires.
 */
static void lowpower_service_idle_timer(void)
{
    if (g_lowpower_state.prepared) {
        return;
    }

    if (!lowpower_idle_timer_allowed()) {
        if (g_lowpower_state.idle_timer_active &&
            !g_lowpower_state.pending) {
            g_lowpower_state.idle_started_tick = HAL_GetTick();
        }
        return;
    }

    if (lowpower_idle_elapsed_ms() >= g_lowpower_state.idle_timeout_ms) {
        g_lowpower_state.quiet_sleep_entry =
            (g_lowpower_state.idle_timeout_ms == LOWPOWER_STOP2_RETURN_TIMEOUT_MS);
        g_lowpower_state.pending = true;
        g_lowpower_state.idle_timer_active = false;
        if (!g_lowpower_state.quiet_sleep_entry) {
            shell_print("\r\n[lowpower] Idle timer expired\r\n");
        }
    }
}

/**
 * @brief Save active power state and shut peripherals down for STOP2 entry.
 * @return true if the system was prepared for sleep, otherwise false.
 */
bool lowpower_prepare_for_sleep(void)
{
    if (g_lowpower_state.prepared) {
        return true;
    }

    g_lowpower_state.filesystem_was_mounted = filesystem_is_mounted();
    g_lowpower_state.sd_power_was_enabled = (HAL_GPIO_ReadPin(SD_PWR_GPIO_Port, SD_PWR_Pin) == GPIO_PIN_SET);
    g_lowpower_state.profile_peripherals_were_up = g_lowpower_state.profile_peripherals_up;
    g_lowpower_state.idle_timer_active = false;
    g_lowpower_state.idle_timeout_ms = LOWPOWER_IDLE_TIMEOUT_MS;

    RTC_Status_t rtc_status = lowpower_program_rtc_wake_timer();
    if (rtc_status != RTC_OK) {
        shell_printf("[lowpower] RTC wake timer setup failed (err=%d)\r\n",
                     rtc_status);
        return false;
    }

    HAL_IWDG_Refresh(&hiwdg);

    if (g_lowpower_state.filesystem_was_mounted) {
        (void)filesystem_unmount();
    }

    /* Keep PB8 record trigger and PB10 RTC event active as wake sources. */
    lowpower_config_output(CLK_OE_GPIO_Port, CLK_OE_Pin, GPIO_PIN_RESET);
    lowpower_rtc_spi_down();
    lowpower_sd_spi_down();
    lowpower_wetlab_uart_down();
    lowpower_optode_uart_down();
    lowpower_ctd_uart_down();
    lowpower_wifi_uart_down();
    lowpower_shell_uart_down();

    g_lowpower_state.profile_peripherals_up = false;
    g_lowpower_state.prepared = true;
    return true;
}

/**
 * @brief Restore peripherals and saved state after returning from STOP2.
 */
void lowpower_restore_from_sleep(void)
{
    if (!g_lowpower_state.prepared) {
        return;
    }

    lowpower_shell_uart_up();

    if (g_lowpower_state.profile_peripherals_were_up) {
        lowpower_rtc_spi_up();
        lowpower_ctd_uart_up();
        lowpower_optode_uart_up();
        lowpower_wetlab_uart_up();
        g_lowpower_state.profile_peripherals_up = true;
    } else {
        lowpower_rtc_spi_down();
        lowpower_ctd_uart_down();
        lowpower_optode_uart_down();
        lowpower_wetlab_uart_down();
        g_lowpower_state.profile_peripherals_up = false;
    }

    if (g_lowpower_state.sd_power_was_enabled) {
        lowpower_sd_spi_up();
    }

    lowpower_config_output(CLK_OE_GPIO_Port, CLK_OE_Pin, GPIO_PIN_RESET);

    /* WiFi is restored by the wake handler for RTC duty-cycle wakes. */
    if (g_lowpower_state.filesystem_was_mounted && g_lowpower_state.sd_power_was_enabled) {
        FS_Result_t fs_status = filesystem_mount();
        if (fs_status != FS_OK && fs_status != FS_ALREADY_MOUNTED) {
            shell_printf("[lowpower] SD remount failed (err=%d)\r\n", fs_status);
            g_app.sd_status = PERIPH_ERROR;
        } else {
            g_app.sd_status = PERIPH_READY;
        }
    }

    g_lowpower_state.prepared = false;
}

/**
 * @brief Main low-power service routine for idle timers, STOP2 entry, and wake handling.
 */
void lowpower_service(void)
{
    lowpower_service_idle_timer();

    if (!g_lowpower_state.pending) {
        return;
    }

    g_lowpower_state.pending = false;
    bool quiet_sleep_entry = g_lowpower_state.quiet_sleep_entry;
    g_lowpower_state.quiet_sleep_entry = false;

    if (quiet_sleep_entry) {
        shell_print("\r\nReturning to STOP2 \r\n");
    } else {
        shell_print("[lowpower] Preparing peripherals for sleep...\r\n");
    }
    if (!lowpower_prepare_for_sleep()) {
        lowpower_enter_idle();
        shell_print(SHELL_PROMPT);
        return;
    }

    HAL_SuspendTick();
    bool rtc_wake = RTC_IsInterruptPending() || RTC_IsInterruptAsserted();
    if (!rtc_wake) {
        if (lowpower_prepare_stop2_entry(&rtc_wake)) {
            HAL_IWDG_Refresh(&hiwdg);
            lowpower_enter_stop2_wfe();
            HAL_IWDG_Refresh(&hiwdg);
            SystemClock_Config();
            HAL_ResumeTick();
            HAL_IWDG_Refresh(&hiwdg);
            (void)lowpower_take_pending_record_trigger();
            rtc_wake = RTC_IsInterruptPending() || RTC_IsInterruptAsserted();
            /* PB8 and PB10 are the only intended STOP2 event wake sources.
             * If WFE returned and RTC is not asserted, treat it as the PB8 edge. */
            if (!rtc_wake && !g_app.start_flag) {
                g_app.start_flag = true;
            }
        } else {
            HAL_ResumeTick();
            HAL_IWDG_Refresh(&hiwdg);
        }
    } else {
        HAL_ResumeTick();
        HAL_IWDG_Refresh(&hiwdg);
    }

    (void)lowpower_stop_rtc_countdown();
    lowpower_restore_from_sleep();
    HAL_IWDG_Refresh(&hiwdg);

    if (g_app.start_flag) {
        lowpower_note_activity();
        shell_print("\r\n[lowpower] Woke from STOP2 (PB8)\r\n");
    } else if (rtc_wake) {
        shell_print("\r\n[lowpower] Woke from STOP2 (RTC)\r\n");
        HAL_IWDG_Refresh(&hiwdg);
        lowpower_start_wifi_duty_cycle();
        HAL_IWDG_Refresh(&hiwdg);
        shell_print(SHELL_PROMPT);
    } else {
        lowpower_enter_idle();
        shell_print("\r\n[lowpower] Woke from STOP2 (unknown)\r\n");
        shell_print(SHELL_PROMPT);
    }
}
