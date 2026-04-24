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
#include "sensors.h"
#include "config.h"
#include "wifi.h"
#include "realtime_comm.h"
#include "lowpower.h"
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

static const char *sys_mode_name(sys_mode_t mode)
{
    switch (mode) {
    case SYS_MODE_IDLE:
        return "IDLE";
    case SYS_MODE_RECORDING:
        return "RECORDING";
    case SYS_MODE_FALSE_START:
        return "FALSE_START";
    case SYS_MODE_TIMEOUT:
        return "TIMEOUT";
    case SYS_MODE_NORMALIZING:
        return "NORMALIZING";
    default:
        return "UNKNOWN";
    }
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
    static const char *wifi_names[]   = {"OFF", "INIT", "STREAMING", "ERROR"};

    shell_printf("Mode:       %s\r\n", sys_mode_name(g_app.mode));
    shell_printf("Uptime:     %lu ms\r\n", HAL_GetTick());
    shell_printf("Tasks:      %u pending\r\n", tasker_pending_count());
    shell_printf("SD/FS:      %s\r\n", periph_names[g_app.sd_status]);
    shell_printf("RTC:        %s\r\n", periph_names[g_app.rtc_status]);
    shell_printf("CTD:        %s\r\n", periph_names[g_app.ctd_status]);
    shell_printf("Optode:     %s\r\n", periph_names[g_app.optode_status]);
    shell_printf("WetLab:     %s\r\n", periph_names[g_app.wetlab_status]);
    shell_printf("WiFi:       %s\r\n", wifi_names[g_app.wifi_state]);
    shell_printf("Sensors:    level %u\r\n", g_app.sensor_level);
}

void handle_low_power_on(const void *arg)
{
    (void)arg;

    if (!lowpower_request_on()) {
        shell_print("[lowpower] Sleep request already pending\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }

    shell_print("[lowpower] Sleep requested\r\n");
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

    RTC_Status_t status = RTC_SetDateTime(&dt);
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
        shell_print("Usage: rtc-settime YYYY MM DD HH MM SS WD\r\n");
        shell_print("  YYYY = 2000..2079\r\n");
        shell_print("  WD = weekday (1=Sun..7=Sat)\r\n");
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

/* Optode Handlers --------------------------------------------------------*/

void handle_optode(const void *arg)
{
    (void)arg;
    optode_data_t data;
    if (optode_sample(&data)) {
        shell_printf("\nOptode:\r\n");
        shell_printf("O2 Concentration: %.3f uM\r\n", data.o2_concentration);
        shell_printf("Temperature:      %.3f C\r\n", data.temperature);
        shell_printf("CalPhase:         %.3f deg\r\n", data.cal_phase);
        shell_printf("TCPhase:          %.3f deg\r\n", data.tc_phase);
        shell_printf("C1RPh:            %.3f deg\r\n", data.c1_rph);
        shell_printf("C2RPh:            %.3f deg\r\n", data.c2_rph);
        shell_printf("C1Amp:            %.1f mV\r\n", data.c1_amp);
        shell_printf("C2Amp:            %.1f mV\r\n", data.c2_amp);
        shell_printf("RawTemp:          %.1f mV\r\n", data.raw_temp);
    } else {
        shell_printf("\nOptode read failed\r\n");
    }
}

void handle_optode_listen(const void *arg)
{
    (void)arg;
    optode_listen();
}

/* WetLab Handlers --------------------------------------------------------*/

void handle_wetlab(const void *arg)
{
    (void)arg;
    wetlab_data_t data;
    if (wetlab_sample(&data)) {
        shell_print("\nWetLab:\r\n");
        shell_printf("  CH1:        %u @ %u nm\r\n",
                     data.ch1_signal, data.ch1_lambda);
        shell_printf("  CH2:        %u @ %u nm\r\n",
                     data.ch2_signal, data.ch2_lambda);
        shell_printf("  CH3:       %u @ %u nm\r\n",
                     data.ch3_signal, data.ch3_lambda);
        shell_printf("  Thermistor: %u\r\n", data.thermistor);
    } else {
        shell_printf("\nWetLab read failed\r\n");
    }
}

void handle_wetlab_raw(const void *arg)
{
    (void)arg;
    wetlab_raw();
}

/* Simultaneous Sampling --------------------------------------------------*/

void handle_sensors(const void *arg)
{
    (void)arg;
    bool has_optode = config_has_optode();
    bool has_wetlab = config_has_wetlab();

    uint32_t t0 = HAL_GetTick();
    sensor_reading_t reading;
    sensors_sample(&reading);
    uint32_t elapsed = HAL_GetTick() - t0;

    shell_printf("[sensors] Time elapsed: %lu ms\r\n", elapsed);

    if (reading.ctd_ok) {
        shell_printf("[CTD] C=%.4f T=%.4f P=%.5f\r\n",
                     reading.ctd.conductivity, reading.ctd.temperature,
                     reading.ctd.pressure);
    } else {
        shell_print("[CTD] FAILED\r\n");
    }

    if (has_optode) {
        if (reading.optode_ok) {
            shell_printf("[Optode] O2=%.3f uM  T=%.3f C  CalPh=%.3f\r\n",
                         reading.optode.o2_concentration,
                         reading.optode.temperature,
                         reading.optode.cal_phase);
        } else {
            shell_print("[Optode] FAILED\r\n");
        }
    }

    if (has_wetlab) {
        if (reading.wetlab_ok) {
            shell_printf("[WetLab] CH1=%u@%unm  CH2=%u@%unm  CH3=%u@%unm  Therm=%u\r\n",
                         reading.wetlab.ch1_signal, reading.wetlab.ch1_lambda,
                         reading.wetlab.ch2_signal, reading.wetlab.ch2_lambda,
                         reading.wetlab.ch3_signal, reading.wetlab.ch3_lambda,
                         reading.wetlab.thermistor);
        } else {
            shell_print("[WetLab] FAILED\r\n");
        }
    }
}

/* Config Handlers --------------------------------------------------------*/

void handle_config(const void *arg)
{
    const config_args_t *a = (const config_args_t *)arg;

    if (a->set) {
        config_set_sensor_level(a->level);
        shell_printf("Sensor config set to %u\r\n", config_get_sensor_level());
    } else {
        uint8_t lvl = config_get_sensor_level();
        shell_printf("Sensor config: %u", lvl);
        switch (lvl) {
            case SENSOR_CFG_CTD_ONLY:   shell_print(" (CTD only)\r\n"); break;
            case SENSOR_CFG_CTD_OPTODE: shell_print(" (CTD + Optode)\r\n"); break;
            case SENSOR_CFG_ALL:        shell_print(" (CTD + Optode + WetLab)\r\n"); break;
            default:                    shell_print(" (unknown)\r\n"); break;
        }
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

/* WiFi Handlers ----------------------------------------------------------*/

void handle_wifi_status(const void *arg)
{
    (void)arg;
    static const char *state_names[] = {
        "OFF", "INIT", "STREAMING", "ERROR"
    };
    wifi_state_t st = wifi_get_state();
    shell_printf("WiFi state: %s (rx buf: %u, rx total: %lu)\r\n",
                 state_names[st], wifi_available(), wifi_get_rx_count());
}

void handle_wifi_up(const void *arg)
{
    (void)arg;
    shell_print("[wifi] Powering up...\r\n");
    wifi_init(&huart4);
    shell_printf("[wifi] Done — state: %s\r\n",
                 wifi_get_state() == WIFI_STATE_STREAMING ? "STREAMING" : "ERROR");
}

void handle_wifi_down(const void *arg)
{
    (void)arg;
    shell_print("[wifi] Powering down...\r\n");
    wifi_down();
    shell_print("[wifi] Done — state: OFF\r\n");
}

void handle_idata(const void *arg)
{
    (void)arg;
    realtime_comm_stream();
}

void handle_who(const void *arg)
{
    (void)arg;
    wifi_printf("%s\r\n> ", DEVICE_SERIAL);
    shell_printf("%s\r\n", DEVICE_SERIAL);
    shell_print(SHELL_PROMPT);
}
