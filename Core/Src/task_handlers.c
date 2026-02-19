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
#include "ctd.h"
#include <string.h>

/* External UART handles declared in main.c */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* Firmware version */
#define FW_VERSION "0.1.0"

/* General Handlers -------------------------------------------------------*/

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
    shell_printf("System uptime: %lu ms\r\n", HAL_GetTick());
    shell_printf("Tasks pending: %u\r\n", tasker_pending_count());
    shell_printf("FS mounted:    %s\r\n", filesystem_is_mounted() ? "yes" : "no");
}

/** @brief  Handle the "reset" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_reset(const void *arg)
{
    const reset_args_t *a = (const reset_args_t *)arg;
    shell_print("System resetting in 3 seconds...\r\n");

    /* Spin until the target tick is reached */
    while (HAL_GetTick() < a->reset_due_ms) {
        /* wait */
    }

    NVIC_SystemReset();
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

/** @brief  Handle the "systime" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_systime(const void *arg)
{
    (void)arg;
    uint32_t tick = HAL_GetTick();
    uint32_t sec = tick / 1000;
    uint32_t min = sec / 60;
    uint32_t hrs = min / 60;
    shell_printf("System tick: %lu ms (%lu:%02lu:%02lu)\r\n",
                 tick, hrs, min % 60, sec % 60);
}

/** @brief  Handle the "hello" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_hello(const void *arg)
{
    const hello_args_t *a = (const hello_args_t *)arg;
    const char *msg = "Hello from STM Prawler!\r\n";
    UART_HandleTypeDef *uart = NULL;

    switch (a->uart_num) {
        case 1: uart = &huart1; break;
        case 2: uart = &huart2; break;
        case 3: uart = &huart3; break;
        case 4: uart = &huart4; break;
        case 5: uart = &huart5; break;
        default:
            shell_print("Invalid UART number (1-5)\r\n");
            return;
    }

    HAL_UART_Transmit(uart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    shell_printf("Sent hello on UART%u\r\n", a->uart_num);
}

/* RTC Handlers -----------------------------------------------------------*/

/** @brief  Handle the "rtc-settime" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_rtc_settime(const void *arg)
{
    const rtc_settime_args_t *a = (const rtc_settime_args_t *)arg;

    if (!a->valid) {
        shell_print("Usage: rtc-settime YYYY MM DD HH MM SS WD\r\n");
        shell_print("  WD = weekday (1=Sun..7=Sat)\r\n");
        return;
    }

    RTC_DateTime_t dt = {0};
    dt.years    = (uint8_t)(a->year % 100);
    dt.months   = a->months;
    dt.days     = a->days;
    dt.hours    = a->hours;
    dt.minutes  = a->minutes;
    dt.seconds  = a->seconds;
    dt.weekdays = a->weekdays;

    RTC_Status_t status = RTC_SetDateTime(&dt);
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

    RTC_Status_t status = RTC_GetDateTime(&dt);
    if (status == RTC_OK) {
        shell_printf("20%02u-%02u-%02u %02u:%02u:%02u (wd=%u)\r\n",
                     dt.years, dt.months, dt.days,
                     dt.hours, dt.minutes, dt.seconds, dt.weekdays);
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

    RTC_Status_t status = RTC_GetTemperature(&temp);
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

    RTC_Status_t status = RTC_SetTimer(&timer);
    if (status == RTC_OK) {
        status = RTC_EnableTimerInterrupt(true);
    }

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

    RTC_Status_t status = RTC_EnableTimer(false);
    if (status == RTC_OK) {
        status = RTC_EnableTimerInterrupt(false);
    }
    if (status == RTC_OK) {
        status = RTC_ClearTimerFlag();
    }

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
    RTC_Status_t status = RTC_GetTimer(&timer);

    if (status == RTC_OK) {
        shell_printf("Timer: %s, value=%u, auto-reload=%s, triggered=%s\r\n",
                     timer.enabled ? "ON" : "OFF",
                     timer.timer_value,
                     timer.auto_reload ? "yes" : "no",
                     RTC_IsTimerTriggered() ? "yes" : "no");
    } else {
        shell_printf("RTC timer status failed (err %d)\r\n", status);
    }
}

/* CTD Handlers -----------------------------------------------------------*/

/** @brief  Handle CTD sensor readings
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_ctd(const void *arg)
{
    (void)arg;
    ctd_data_t data;
    if (ctd_ts(&data)) {
        shell_print("\nCTD Sensor Readings:\r\n");
        shell_printf("Conductivity: %.4f\n", data.conductivity);
        shell_printf("Temperature:  %.4f\n", data.temperature);
        shell_printf("Pressure:     %.5f\r\n", data.pressure);
    } else {
        shell_printf("\nCTD read failed\r\n");
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
    FS_Result_t res = filesystem_mount();
    switch (res) {
        case FS_OK:
            shell_print("File system mounted\r\n");
            break;
        case FS_ALREADY_MOUNTED:
            shell_print("File system already mounted\r\n");
            break;
        default:
            shell_printf("Mount failed (err %d)\r\n", res);
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
}

/** @brief  Handle the "fs-df" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_df(const void *arg)
{
    (void)arg;
    uint32_t total, free_bytes, used_pct;

    FS_Result_t res = filesystem_df(&total, &free_bytes, &used_pct);
    if (res == FS_OK) {
        shell_printf("Total: %lu bytes\r\n", total);
        shell_printf("Free:  %lu bytes\r\n", free_bytes);
        shell_printf("Used:  %lu%%\r\n", used_pct);
    } else if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else {
        shell_printf("df failed (err %d)\r\n", res);
    }
}

/** @brief  Handle the "fs-ls" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_ls(const void *arg)
{
    (void)arg;
    FS_Result_t res = filesystem_ls(shell_print);
    if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else if (res != FS_OK) {
        shell_printf("ls failed (err %d)\r\n", res);
    }
}

/** @brief  Handle the "fs-cat" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_cat(const void *arg)
{
    (void)arg;
    FS_Buffers_t *buf = filesystem_get_buffers();

    if (buf->filename[0] == '\0') {
        shell_print("Usage: fs-cat <filename>\r\n");
        return;
    }

    FS_Result_t res = filesystem_cat(buf->filename, shell_print);
    if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else if (res == FS_FILE_NOT_FOUND) {
        shell_printf("File not found: %s\r\n", buf->filename);
    } else if (res != FS_OK) {
        shell_printf("cat failed (err %d)\r\n", res);
    }
    shell_print("\r\n");
}

/** @brief  Handle the "fs-write" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_write(const void *arg)
{
    (void)arg;
    FS_Buffers_t *buf = filesystem_get_buffers();

    if (buf->filename[0] == '\0' || buf->file_data[0] == '\0') {
        shell_print("Usage: fs-write <filename> <data>\r\n");
        return;
    }

    FS_Result_t res = filesystem_write(buf->filename, buf->file_data);
    if (res == FS_OK) {
        shell_printf("Written to %s\r\n", buf->filename);
    } else if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else {
        shell_printf("write failed (err %d)\r\n", res);
    }
}

/** @brief  Handle the "fs-rm" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_rm(const void *arg)
{
    (void)arg;
    FS_Buffers_t *buf = filesystem_get_buffers();

    if (buf->filename[0] == '\0') {
        shell_print("Usage: fs-rm <filename>\r\n");
        return;
    }

    FS_Result_t res = filesystem_rm(buf->filename);
    if (res == FS_OK) {
        shell_printf("Deleted %s\r\n", buf->filename);
    } else if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else if (res == FS_FILE_NOT_FOUND) {
        shell_printf("File not found: %s\r\n", buf->filename);
    } else {
        shell_printf("rm failed (err %d)\r\n", res);
    }
}

/** @brief  Handle the "fs-mkdir" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_mkdir(const void *arg)
{
    (void)arg;
    FS_Buffers_t *buf = filesystem_get_buffers();

    if (buf->dirname[0] == '\0') {
        shell_print("Usage: fs-mkdir <dirname>\r\n");
        return;
    }

    FS_Result_t res = filesystem_mkdir(buf->dirname);
    if (res == FS_OK) {
        shell_printf("Created directory %s\r\n", buf->dirname);
    } else if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else {
        shell_printf("mkdir failed (err %d)\r\n", res);
    }
}

/** @brief  Handle the "fs-rmdir" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_rmdir(const void *arg)
{
    (void)arg;
    FS_Buffers_t *buf = filesystem_get_buffers();

    if (buf->dirname[0] == '\0') {
        shell_print("Usage: fs-rmdir <dirname>\r\n");
        return;
    }

    FS_Result_t res = filesystem_rmdir(buf->dirname);
    if (res == FS_OK) {
        shell_printf("Removed directory %s\r\n", buf->dirname);
    } else if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else {
        shell_printf("rmdir failed (err %d)\r\n", res);
    }
}

/** @brief  Handle the "fs-cp" command
  * @param  arg: Pointer to arguments (not used)
  * @retval None
  */
void handle_fs_cp(const void *arg)
{
    (void)arg;
    FS_Buffers_t *buf = filesystem_get_buffers();

    if (buf->filename[0] == '\0' || buf->dest_filename[0] == '\0') {
        shell_print("Usage: fs-cp <source> <dest>\r\n");
        return;
    }

    FS_Result_t res = filesystem_cp(buf->filename, buf->dest_filename);
    if (res == FS_OK) {
        shell_printf("Copied %s -> %s\r\n", buf->filename, buf->dest_filename);
    } else if (res == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else if (res == FS_FILE_NOT_FOUND) {
        shell_printf("Source not found: %s\r\n", buf->filename);
    } else {
        shell_printf("cp failed (err %d)\r\n", res);
    }
}
