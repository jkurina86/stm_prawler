/**
  ******************************************************************************
  * @file    task_handlers.c
  * @brief   Deferred task handler implementations
  * @note    All handlers run in main loop context (not ISR).
  *          They use shell_printf/shell_print for output.
  ******************************************************************************
  */

#include "task_handlers.h"
#include "shell.h"
#include "main.h"
#include "tasker.h"
#include "filesystem.h"
#include "ab-rtcmc-rtc.h"
#include "config.h"
#include "lowpower.h"
#include "sd_spi.h"
#include "LI8x0.h"
#include <string.h>

/* Firmware version */
#define FW_VERSION "0.1.0"

/* General Handlers -------------------------------------------------------*/

static void print_sd_diag(void)
{
    const SD_SPI_Diag_t *diag = USER_SPI_diag();

    shell_printf("[sd] stage=%s stat=0x%02x type=0x%02x cmd=%u resp=0x%02x "
                 "token=0x%02x sector=%lu count=%u hal=%u spierr=0x%08lx dres=%d\r\n",
                 USER_SPI_diag_stage_name(diag->stage),
                 diag->stat,
                 diag->card_type,
                 diag->last_cmd,
                 diag->last_resp,
                 diag->last_token,
                 (unsigned long)diag->last_sector,
                 diag->last_count,
                 diag->last_hal_status,
                 (unsigned long)diag->spi_error,
                 diag->last_result);
}

/** @brief  Handle the "help" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_help(const void *arg)
{
    (void)arg;
    shell_print("\r\nAvailable commands:\r\n");
    shell_print("-------------------\r\n");
    for (int i = 0; shell_commands[i].name != NULL; i++) {
        shell_printf("  %-18s %s\r\n", shell_commands[i].name, shell_commands[i].description);
    }
}

/** @brief  Handle the "clear" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_clear(const void *arg)
{
    (void)arg;
    shell_print("\033[2J\033[H"); /* ANSI clear screen + cursor home */
}

/** @brief  Handle the "status" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_status(const void *arg)
{
    (void)arg;
    static const char *periph_names[] = {"OFF", "READY", "ERROR"};

    shell_print("Mode:       SHELL\r\n");
    shell_printf("Uptime:     %lu ms\r\n", (unsigned long)HAL_GetTick());
    shell_printf("Tasks:      %u pending\r\n", tasker_pending_count());
    shell_printf("SD/FS:      %s\r\n", periph_names[g_app.sd_status]);
    shell_printf("RTC:        %s\r\n", periph_names[g_app.rtc_status]);
    shell_printf("LI-COR:     %s (USART2, 9600 8N1)\r\n", periph_names[g_app.licor_status]);
    li8x0_model_t model = li8x0_get_model();
    if (model == LI8X0_UNKNOWN)
        shell_print("LI-COR model: undiscovered\r\n");
    else
        shell_printf("LI-COR model: LI-%u\r\n", (unsigned)model);
}

/** @brief  Handle the "version" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_version(const void *arg)
{
    (void)arg;
    shell_printf("Firmware version: %s\r\n", FW_VERSION);
}

/* RTC Handlers -----------------------------------------------------------*/

/** @brief  Handle the "settime" command (Unix epoch)
  * @param  arg: Pointer to settime_args_t
  * @retval None
  */
void handle_settime(const void *arg)
{
    const settime_args_t *a = (const settime_args_t *)arg;

    if (a->unix_epoch == 0) {
        shell_print("Usage: settime <UNIX_EPOCH>\r\n");
        return;
    }

    RTC_DateTime_t dt = {0};
    RTC_FromUnixEpoch(a->unix_epoch, &dt);

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_SetDateTime(&dt);
    lowpower_rtc_end(release_spi);
    if (status == RTC_OK) {
        shell_printf("RTC set to 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
                     dt.years, dt.months, dt.days,
                     dt.hours, dt.minutes, dt.seconds);
    } else {
        shell_printf("RTC set time failed (err %d)\r\n", status);
    }
}

/** @brief  Handle the "rtc-settime" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_settime(const void *arg)
{
    const rtc_settime_args_t *a = (const rtc_settime_args_t *)arg;

    if (!a->valid) {
        shell_print("Usage: rtc-settime YYYY MM DD HH MM SS\r\n");
        shell_print("  YYYY = 2000..2079\r\n");
        return;
    }

    if (a->year < 2000 || a->year > 2079) {
        shell_print("RTC year must be 2000..2079\r\n");
        return;
    }

    RTC_DateTime_t dt = {0};
    dt.years    = (uint8_t)(a->year - 2000);
    dt.months   = a->months;
    dt.days     = a->days;
    dt.hours    = a->hours;
    dt.minutes  = a->minutes;
    dt.seconds  = a->seconds;

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_SetDateTime(&dt);
    lowpower_rtc_end(release_spi);
    if (status == RTC_OK) {
        shell_printf("RTC set to 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
                     dt.years, dt.months, dt.days,
                     dt.hours, dt.minutes, dt.seconds);
    } else {
        shell_printf("RTC set time failed (err %d)\r\n", status);
    }
}

/** @brief  Handle the "rtc-gettime" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_gettime(const void *arg)
{
    (void)arg;
    RTC_DateTime_t dt = {0};

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_GetDateTime(&dt);
    lowpower_rtc_end(release_spi);
    if (status == RTC_OK) {
        shell_printf("20%02u-%02u-%02u %02u:%02u:%02u\r\n",
                     dt.years, dt.months, dt.days,
                     dt.hours, dt.minutes, dt.seconds);
    } else {
        shell_printf("RTC read failed (err %d)\r\n", status);
    }
}

/** @brief  Handle the "rtc-epoch" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_epoch(const void *arg)
{
    (void)arg;
    RTC_DateTime_t dt = {0};

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_GetDateTime(&dt);
    lowpower_rtc_end(release_spi);
    if (status == RTC_OK) {
        uint32_t unix_epoch = RTC_ToUnixEpoch(&dt);
        shell_printf("%lu\r\n", (unsigned long)unix_epoch);
    } else {
        shell_printf("RTC read failed (err %d)\r\n", status);
    }
}

/** @brief  Handle the "rtc-temp" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_temp(const void *arg)
{
    (void)arg;
    int8_t temp;

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_GetTemperature(&temp);
    lowpower_rtc_end(release_spi);
    if (status == RTC_OK) {
        shell_printf("RTC temperature: %d C\r\n", temp);
    } else {
        shell_printf("RTC temp read failed (err %d)\r\n", status);
    }
}

/** @brief  Handle the "rtc-timer-set" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_timer_set(const void *arg)
{
    const rtc_timer_set_args_t *a = (const rtc_timer_set_args_t *)arg;

    if (a->seconds == 0) {
        shell_print("Usage: rtc-timer-set <seconds>\r\n");
        return;
    }

    RTC_Timer_t timer = {0};
    timer.timer_value = a->seconds;
    timer.division    = RTC_TIMER_DIV_1HZ;
    timer.auto_reload = false;
    timer.enabled     = true;

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_SetTimer(&timer);
    if (status == RTC_OK) {
        status = RTC_EnableTimerInterrupt(true);
    }
    lowpower_rtc_end(release_spi);

    if (status == RTC_OK) {
        shell_printf("RTC timer set to %u s\r\n", a->seconds);
    } else {
        shell_printf("RTC timer set failed (err %d)\r\n", status);
    }
}

/** @brief  Handle the "rtc-timer-stop" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_timer_stop(const void *arg)
{
    (void)arg;

    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_EnableTimer(false);
    if (status == RTC_OK) {
        status = RTC_EnableTimerInterrupt(false);
    }
    if (status == RTC_OK) {
        status = RTC_ClearTimerFlag();
    }
    lowpower_rtc_end(release_spi);

    if (status == RTC_OK) {
        shell_print("RTC timer stopped\r\n");
    } else {
        shell_printf("RTC timer stop failed (err %d)\r\n", status);
    }
}

/** @brief  Handle RTC timer status
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_timer_status(const void *arg)
{
    (void)arg;

    RTC_Timer_t timer = {0};
    bool triggered = false;
    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_GetTimer(&timer);
    if (status == RTC_OK) {
        triggered = RTC_IsTimerTriggered();
    }
    lowpower_rtc_end(release_spi);

    if (status == RTC_OK) {
        shell_printf("Timer: %s, value=%u, auto-reload=%s, triggered=%s\r\n",
                     timer.enabled ? "ON" : "OFF",
                     timer.timer_value,
                     timer.auto_reload ? "yes" : "no",
                     triggered ? "yes" : "no");
    } else {
        shell_printf("RTC timer status failed (err %d)\r\n", status);
    }
}

/* File System Handlers ---------------------------------------------------*/

/** @brief  Handle the "fs-mount" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_mount(const void *arg)
{
    (void)arg;

    lowpower_sd_spi_up();
    FS_Result_t res = filesystem_mount();
    switch (res) {
        case FS_OK:
            g_app.sd_status = PERIPH_READY;
            shell_print("File system mount OK\r\n");
            (void)filesystem_unmount();
            lowpower_sd_spi_down();
            break;
        case FS_ALREADY_MOUNTED:
            g_app.sd_status = PERIPH_READY;
            shell_print("File system already mounted\r\n");
            (void)filesystem_unmount();
            lowpower_sd_spi_down();
            break;
        default:
            g_app.sd_status = PERIPH_ERROR;
            shell_printf("Mount failed (err %d fatfs %d)\r\n",
                         res, filesystem_last_fatfs_result());
            print_sd_diag();
            lowpower_sd_spi_down();
            break;
    }
}

/** @brief  Handle the "fs-unmount" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_unmount(const void *arg)
{
    (void)arg;
    FS_Result_t res = filesystem_unmount();
    switch (res) {
        case FS_OK:
            shell_print("File system unmounted\r\n");
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        default:
            shell_printf("Unmount failed (err %d)\r\n", res);
            break;
    }

    lowpower_sd_spi_down();
}

/** @brief  Handle the "fs-ls" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_ls(const void *arg)
{
    (void)arg;
    lowpower_sd_spi_up();
    FS_Result_t res = filesystem_mount();
    if (res != FS_OK && res != FS_ALREADY_MOUNTED) {
        g_app.sd_status = PERIPH_ERROR;
        shell_printf("Mount failed (err %d fatfs %d)\r\n",
                     res, filesystem_last_fatfs_result());
        print_sd_diag();
        lowpower_sd_spi_down();
        return;
    }

    g_app.sd_status = PERIPH_READY;
    res = filesystem_ls(shell_print);
    if (res != FS_OK) {
        shell_printf("ls failed (err %d)\r\n", res);
    }

    (void)filesystem_unmount();
    lowpower_sd_spi_down();
}

/* Report identity on the UART shell. */
void handle_who(const void *arg)
{
    (void)arg;
    shell_printf("%s\r\n", DEVICE_SERIAL);
}
