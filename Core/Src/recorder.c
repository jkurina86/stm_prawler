/**
  ******************************************************************************
  * @file    recorder.c
  * @brief   PB8-triggered continuous sensor logging to SD card
  * @note    When PB8 goes HIGH, samples all 3 sensors every 4s and writes
  *          CSV records to a double-buffered log file. When PB8 goes LOW,
  *          flushes remaining data and closes the file.
  ******************************************************************************
  */

#include "recorder.h"
#include "main.h"
#include "shell.h"
#include "filesystem.h"
#include "ab-rtcmc-rtc.h"
#include "sensors.h"
#include "stm32l4xx_hal.h"
#include "config.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define REC_BUF_SIZE          1024
#define SAMPLE_INTERVAL       4000  /* ms between samples */
#define TOLERANCE             0.5f  /* Depth tolerance (dbar) */
#define FALSE_START_SAMPLES   15    /* Validation window: 15 × 4s = 60s */
#define NORM_INTERVAL         20000 /* ms between normalization samples */
#define NORM_SAMPLES          30    /* ~10 min normalization */
#define DEBOUNCE_MS           100   /* PB8 debounce period */

/* Private types -------------------------------------------------------------*/
typedef enum {
    REC_IDLE,
    REC_RECORDING,
    REC_TIMEOUT,
    REC_NORMALIZING
} rec_state_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t rec_buf[2][REC_BUF_SIZE];
static uint8_t active_buf;
static uint16_t buf_offset;

static rec_state_t state;
static uint32_t start_time;
static float initial_depth = 0.0f;
static float max_pressure;
static uint16_t sample_count;
static uint32_t record_num;
static uint32_t next_sample_tick;
static char rec_filename[32];
static char last_successful_filename[32];

static sensor_reading_t reading;
static uint16_t norm_count;
static uint32_t debounce_tick;
static bool     debounce_active;

/* Private functions ---------------------------------------------------------*/

static uint32_t get_timestamp(void)
{
    RTC_DateTime_t dt = {0};
    if (RTC_GetDateTime(&dt) != RTC_OK) {
        return 0;
    }
    return RTC_ToGPSEpoch(&dt);
}

static bool flush_buffer(const uint8_t *buf, uint16_t len)
{
    if (len == 0)
        return true;

    if (filesystem_log_write(buf, len) != FS_OK) {
        shell_print("[recorder] Write error\r\n");
        return false;
    }

    if (filesystem_log_sync() != FS_OK) {
        shell_print("[recorder] Sync error\r\n");
        return false;
    }

    return true;
}

static bool open_log_file(void)
{
    if (!filesystem_is_mounted()) {
        shell_print("[recorder] FS not mounted\r\n");
        return false;
    }

    RTC_DateTime_t dt = {0};
    RTC_GetDateTime(&dt);

    snprintf(rec_filename, sizeof(rec_filename),
             "%02u%02u%04u_%02u%02u%02u_record.csv",
             dt.days, dt.months, 2000 + dt.years,
             dt.hours, dt.minutes, dt.seconds);

    FS_Result_t res = filesystem_log_create(rec_filename);
    if (res != FS_OK) {
        shell_printf("[recorder] Cannot create %s (err=%d)\r\n", rec_filename, res);
        return false;
    }

    /* Write CSV header */
    static const char hdr[] =
        "ProfileNo,GPS_Epoch_UTC,CTD_C,CTD_T,CTD_D,"
        "Optode_O2,Optode_Temp,Optode_Cal_Ph,Optode_Tc_Ph,"
        "Optode_C1_Ph,Optode_C2_Ph,Optode_C1_Amp,Optode_C2_Amp,Optode_Temp_raw,"
        "Wetlab_C1_lambda,Wetlab_C1_signal,Wetlab_C2_lambda,Wetlab_C2_signal,"
        "Wetlab_C3_lambda,Wetlab_C3_signal,Wetlab_Therm\r\n";
    filesystem_log_write((const uint8_t *)hdr, sizeof(hdr) - 1);
    filesystem_log_sync();
    return true;
}

static void sample_and_record(void)
{
    sensors_sample(&reading);

    /* Format CSV line */
    uint32_t ts = get_timestamp();
    char line_buf[256];
    int line_len = snprintf(line_buf, sizeof(line_buf),
        "%lu,%lu,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%u,%u,%u,%u,%u,%u,%u\r\n",
        record_num, ts,
        reading.ctd.conductivity, reading.ctd.temperature, reading.ctd.pressure,
        reading.optode.o2_concentration, reading.optode.temperature,
        reading.optode.cal_phase, reading.optode.tc_phase,
        reading.optode.c1_rph, reading.optode.c2_rph,
        reading.optode.c1_amp, reading.optode.c2_amp, reading.optode.raw_temp,
        reading.wetlab.chl_lambda, reading.wetlab.chl_signal,
        reading.wetlab.ntu_lambda, reading.wetlab.ntu_signal,
        reading.wetlab.cdom_lambda, reading.wetlab.cdom_signal,
        reading.wetlab.thermistor);

    if (line_len < 0 || (size_t)line_len >= sizeof(line_buf))
        line_len = sizeof(line_buf) - 1;

    /* Check if line fits in current buffer */
    if (buf_offset + line_len > REC_BUF_SIZE) {
        flush_buffer(rec_buf[active_buf], buf_offset);
        active_buf ^= 1;
        buf_offset = 0;
    }

    memcpy(&rec_buf[active_buf][buf_offset], line_buf, line_len);
    buf_offset += line_len;
    record_num++;
}

/**
 * @brief  Debounced PB8 LOW detection.
 * @retval true when PB8 has been continuously LOW for DEBOUNCE_MS.
 */
static bool pb8_low_debounced(void)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_RESET) {
        debounce_active = false;
        return false;
    }
    if (!debounce_active) {
        debounce_active = true;
        debounce_tick = HAL_GetTick();
        return false;
    }
    return (HAL_GetTick() - debounce_tick) >= DEBOUNCE_MS;
}

static uint16_t get_max_samples(void)
{
    switch (g_app.sensor_level) {
    case SENSOR_CFG_CTD_ONLY:   return MEASUREMENTS_CTD_ONLY;
    case SENSOR_CFG_CTD_OPTODE: return MEASUREMENTS_CTD_OPTODE;
    default:                    return MEASUREMENTS_CFG_ALL;
    }
}

static void enter_normalization(void)
{
    debounce_active = false;
    norm_count = 0;
    next_sample_tick = HAL_GetTick() + NORM_INTERVAL;
    state = REC_NORMALIZING;
    g_app.mode = SYS_MODE_NORMALIZING;
    shell_printf("[recorder] Normalization started (%u samples)\r\n",
                 NORM_SAMPLES);
}

/* Public functions ----------------------------------------------------------*/

void recorder_init(void)
{
    state = REC_IDLE;
    active_buf = 0;
    buf_offset = 0;
    record_num = 0;

    /* Seed last filename from SD card if a recording exists */
    if (filesystem_find_latest("_record.csv",
            last_successful_filename,
            sizeof(last_successful_filename)) == FS_OK) {
        shell_printf("[recorder] Found previous recording: %s\r\n",
                     last_successful_filename);
    }
}

void recorder_service(void)
{
    switch (state) {

    case REC_IDLE:
        if (g_app.start_flag) {
            g_app.start_flag = false;
            debounce_active = true;
            debounce_tick = HAL_GetTick();
            break;
        }

        /* Wait for PB8 to remain HIGH through debounce period */
        if (debounce_active) {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_SET) {
                debounce_active = false;
                break;
            }
            if ((HAL_GetTick() - debounce_tick) < DEBOUNCE_MS)
                break;
            debounce_active = false;

            if (!open_log_file())
                return;

            record_num = 0;
            buf_offset = 0;
            active_buf = 0;

            shell_printf("[recorder] Started: %s\r\n", rec_filename);

            /* Take first sample immediately */
            sample_and_record();
            sample_count = 0;
            next_sample_tick = HAL_GetTick() + SAMPLE_INTERVAL;
            start_time = HAL_GetTick();
            initial_depth = reading.ctd.pressure;
            max_pressure = reading.ctd.pressure;

            state = REC_RECORDING;
            g_app.mode = SYS_MODE_RECORDING;
        }
        break;

    case REC_RECORDING:
        /* Check if PB8 went LOW (debounced) — stop recording, enter normalization */
        if (pb8_low_debounced()) {
            flush_buffer(rec_buf[active_buf], buf_offset);
            buf_offset = 0;

            shell_printf("[recorder] PB8 LOW after %lu records\r\n", record_num);
            enter_normalization();
            return;
        }

        /* Time for next sample? */
        if ((HAL_GetTick() - next_sample_tick) < 0x80000000UL) {
            sample_and_record();
            sample_count++;
            next_sample_tick += SAMPLE_INTERVAL;

            /* False start detection during validation window */
            if (sample_count <= FALSE_START_SAMPLES) {
                if (reading.ctd.pressure > max_pressure)
                    max_pressure = reading.ctd.pressure;

                if (sample_count == FALSE_START_SAMPLES) {
                    if (max_pressure <= initial_depth + TOLERANCE) {
                        /* No significant descent — discard recording */
                        filesystem_log_close();
                        filesystem_log_delete(rec_filename);
                        shell_printf("[recorder] False start — file %s removed\r\n",
                                     rec_filename);
                        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
                        g_app.start_flag = false;
                        state = REC_IDLE;
                        g_app.mode = SYS_MODE_IDLE;
                        start_time = 0;
                        return;
                    }
                    /* Passed false-start check — record filename for realtime */
                    strncpy(last_successful_filename, rec_filename,
                            sizeof(last_successful_filename) - 1);
                    last_successful_filename[sizeof(last_successful_filename) - 1] = '\0';
                }
            }

            /* Recording timeout — max samples reached, wait for PB8 LOW */
            if (sample_count >= get_max_samples()) {
                flush_buffer(rec_buf[active_buf], buf_offset);
                buf_offset = 0;

                shell_printf("[recorder] Timeout at %u samples, waiting for PB8 LOW\r\n",
                             sample_count);
                debounce_active = false;
                state = REC_TIMEOUT;
                g_app.mode = SYS_MODE_TIMEOUT;
                return;
            }
        }

        break;

    case REC_TIMEOUT:
        /* Wait for PB8 to go LOW (debounced) before starting normalization */
        if (g_app.start_flag)
            g_app.start_flag = false;

        if (pb8_low_debounced()) {
            shell_printf("[recorder] PB8 LOW after timeout\r\n");
            enter_normalization();
        }
        break;

    case REC_NORMALIZING:
        /* Ignore PB8 triggers during normalization */
        if (g_app.start_flag)
            g_app.start_flag = false;

        if ((HAL_GetTick() - next_sample_tick) < 0x80000000UL) {
            sample_and_record();
            norm_count++;
            next_sample_tick += NORM_INTERVAL;

            if (norm_count >= NORM_SAMPLES) {
                flush_buffer(rec_buf[active_buf], buf_offset);
                filesystem_log_close();

                shell_printf("[recorder] Complete. %lu records in %s\r\n",
                             record_num, rec_filename);

                __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
                g_app.start_flag = false;
                state = REC_IDLE;
                g_app.mode = SYS_MODE_IDLE;
                start_time = 0;
            }
        }
        break;
    }
}

const char *recorder_get_last_filename(void)
{
    return last_successful_filename[0] ? last_successful_filename : NULL;
}
