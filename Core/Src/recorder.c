/**
  ******************************************************************************
  * @file    recorder.c
  * @brief   PB8-triggered sensor logging with in-RAM buffering.
  * @note    On PB8 rising edge, brings up the profile sensor path, powers
  *          WiFi down, and buffers every sample into a static profile_data_t.
  *          When normalization finishes, the SD is mounted only long enough
  *          to write the CSV, then the idle power policy is restored.
  ******************************************************************************
  */

#include "recorder.h"
#include "main.h"
#include "shell.h"
#include "filesystem.h"
#include "ab-rtcmc-rtc.h"
#include "sensors.h"
#include "profile_data.h"
#include "realtime_comm.h"
#include "lowpower.h"
#include "stm32l4xx_hal.h"
#include "config.h"

#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define SAMPLE_INTERVAL       4000  /* ms between samples */
#define TOLERANCE             0.5f  /* Depth tolerance (dbar) */
#define FALSE_START_SAMPLES   15    /* Validation window: 15 × 4s = 60s */
#define NORM_INTERVAL         20000 /* ms between normalization samples */
#define NORM_SAMPLES          PROFILE_NORM_SAMPLES
#define DEBOUNCE_MS           100   /* PB8 debounce period */
#define PROFILE_BRINGUP_MS    3000  /* Instrument bringup before sampling */

/* Private types -------------------------------------------------------------*/
typedef enum {
    REC_IDLE,
    REC_BRINGUP,
    REC_RECORDING,
    REC_TIMEOUT,
    REC_NORMALIZING
} rec_state_t;

/* Private variables ---------------------------------------------------------*/
static profile_data_t g_profile;

static rec_state_t state;
static uint32_t start_time;
static float initial_depth = 0.0f;
static float max_pressure;
static uint16_t sample_count;
static uint32_t next_sample_tick;
static uint32_t bringup_tick;
static char rec_filename[32];
static char last_successful_filename[32];

static sensor_reading_t reading;
static uint16_t norm_count;
static uint32_t debounce_tick;
static bool     debounce_active;
static GPIO_PinState pb8_last_state;
static bool     pb8_state_valid;

/* Private functions ---------------------------------------------------------*/

static GPIO_PinState pb8_read(void)
{
    return HAL_GPIO_ReadPin(RECORD_TRIGGER_GPIO_Port, RECORD_TRIGGER_Pin);
}

static void pb8_sync_state(void)
{
    pb8_last_state = pb8_read();
    pb8_state_valid = true;
}

static bool pb8_take_start_request(void)
{
    bool requested = false;
    GPIO_PinState current = pb8_read();

    if (g_app.start_flag) {
        g_app.start_flag = false;
        requested = true;
    }

    if (__HAL_GPIO_EXTI_GET_IT(RECORD_TRIGGER_Pin) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(RECORD_TRIGGER_Pin);
        requested = true;
    }

    if (!pb8_state_valid) {
        pb8_last_state = current;
        pb8_state_valid = true;
    } else {
        if (pb8_last_state == GPIO_PIN_RESET && current == GPIO_PIN_SET) {
            requested = true;
        }
        pb8_last_state = current;
    }

    if (!requested || current != GPIO_PIN_SET) {
        return false;
    }

    lowpower_note_activity();
    return true;
}

static void pb8_discard_start_request(void)
{
    g_app.start_flag = false;
    __HAL_GPIO_EXTI_CLEAR_IT(RECORD_TRIGGER_Pin);
    pb8_sync_state();
}

static uint32_t get_unix_timestamp(void)
{
    RTC_DateTime_t dt = {0};
    bool release_spi = lowpower_rtc_begin();
    RTC_Status_t status = RTC_GetDateTime(&dt);
    lowpower_rtc_end(release_spi);

    if (status != RTC_OK) {
        return 0;
    }
    return RTC_ToUnixEpoch(&dt);
}

static bool sd_mount_for_flush(void)
{
    lowpower_sd_spi_up();

    FS_Result_t res = filesystem_mount();
    if (res != FS_OK && res != FS_ALREADY_MOUNTED) {
        shell_printf("[recorder] Mount failed (err=%d)\r\n", res);
        g_app.sd_status = PERIPH_ERROR;
        lowpower_sd_spi_down();
        return false;
    }

    g_app.sd_status = PERIPH_READY;
    return true;
}

static void return_to_idle(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(RECORD_TRIGGER_Pin);
    g_app.start_flag = false;
    pb8_sync_state();
    state = REC_IDLE;
    start_time = 0;
    bringup_tick = 0;
    debounce_active = false;

    lowpower_profile_peripherals_down();
    lowpower_wifi_start();
    lowpower_enter_idle();
}

static int format_measurement_csv(char *buf, size_t buf_size,
                                  const measurement_data_t *m,
                                  uint32_t measurement_num)
{
    return snprintf(buf, buf_size,
        "%lu,%lu,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%u,%u,%u,%u,%u,%u,%u\r\n",
        measurement_num, (unsigned long)m->timestamp,
        m->ctd.conductivity, m->ctd.temperature, m->ctd.pressure,
        m->optode.o2_concentration, m->optode.temperature,
        m->optode.cal_phase, m->optode.tc_phase,
        m->optode.c1_rph, m->optode.c2_rph,
        m->optode.c1_amp, m->optode.c2_amp, m->optode.raw_temp,
        m->wetlab.ch1_lambda, m->wetlab.ch1_signal,
        m->wetlab.ch2_lambda, m->wetlab.ch2_signal,
        m->wetlab.ch3_lambda, m->wetlab.ch3_signal,
        m->wetlab.thermistor);
}

static bool build_filename(uint32_t start_epoch, char *out, size_t out_size)
{
    RTC_DateTime_t dt = {0};
    RTC_FromUnixEpoch(start_epoch, &dt);

    int n = snprintf(out, out_size,
                     "%02u%02u%04u_%02u%02u%02u_record.csv",
                     dt.days, dt.months, 2000 + dt.years,
                     dt.hours, dt.minutes, dt.seconds);
    return (n > 0 && (size_t)n < out_size);
}

/**
 * @brief  Flush the profile buffer to a single CSV file on SD.
 *         Assumes the SD is already powered on and mounted.
 */
static bool flush_profile_to_sd(void)
{
    if (!build_filename(g_profile.start_epoch, rec_filename, sizeof(rec_filename))) {
        shell_print("[recorder] Filename build failed\r\n");
        return false;
    }

    FS_Result_t res = filesystem_log_create(rec_filename);
    if (res != FS_OK) {
        shell_printf("[recorder] Cannot create %s (err=%d)\r\n", rec_filename, res);
        return false;
    }

    static const char hdr[] =
        "MeasurementNo,Unix_Epoch_UTC,CTD_C,CTD_T,CTD_D,"
        "Optode_O2,Optode_Temp,Optode_Cal_Ph,Optode_Tc_Ph,"
        "Optode_C1_Ph,Optode_C2_Ph,Optode_C1_Amp,Optode_C2_Amp,Optode_Temp_raw,"
        "Wetlab_C1_lambda,Wetlab_C1_signal,Wetlab_C2_lambda,Wetlab_C2_signal,"
        "Wetlab_C3_lambda,Wetlab_C3_signal,Wetlab_Therm\r\n";
    if (filesystem_log_write((const uint8_t *)hdr, sizeof(hdr) - 1) != FS_OK) {
        filesystem_log_close();
        return false;
    }

    char line_buf[256];
    for (uint16_t i = 0; i < g_profile.count; i++) {
        int len = format_measurement_csv(line_buf, sizeof(line_buf),
                                         &g_profile.measurements[i], i + 1);
        if (len < 0 || (size_t)len >= sizeof(line_buf))
            len = sizeof(line_buf) - 1;

        if (filesystem_log_write((const uint8_t *)line_buf, (uint16_t)len) != FS_OK) {
            shell_printf("[recorder] Write error at row %u\r\n", i);
            filesystem_log_close();
            return false;
        }
    }

    filesystem_log_sync();
    filesystem_log_close();
    return true;
}

static void record_sample(uint32_t interval_ms)
{
    if (g_profile.count >= PROFILE_MAX_MEASUREMENTS) {
        next_sample_tick += interval_ms;
        return;
    }

    sensors_sample(&reading);

    measurement_data_t *m = &g_profile.measurements[g_profile.count++];
    m->timestamp = get_unix_timestamp();
    m->ctd       = reading.ctd;
    m->optode    = reading.optode;
    m->wetlab    = reading.wetlab;

    next_sample_tick += interval_ms;
}

/**
 * @brief  Debounced PB8 LOW detection.
 * @retval true when PB8 has been continuously LOW for DEBOUNCE_MS.
 */
static bool pb8_low_debounced(void)
{
    if (pb8_read() != GPIO_PIN_RESET) {
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
    g_profile.count = 0;
    g_profile.sensor_level = SENSOR_CFG_ALL;
    g_profile.start_epoch = 0;
    pb8_discard_start_request();

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
        if (!debounce_active && pb8_take_start_request()) {
            debounce_active = true;
            debounce_tick = HAL_GetTick();
            break;
        }

        /* Wait for PB8 to remain HIGH through debounce period */
        if (debounce_active) {
            if (pb8_read() != GPIO_PIN_SET) {
                debounce_active = false;
                pb8_sync_state();
                break;
            }
            if ((HAL_GetTick() - debounce_tick) < DEBOUNCE_MS)
                break;
            debounce_active = false;

            g_profile.count = 0;
            g_profile.sensor_level = g_app.sensor_level;
            g_profile.start_epoch = 0;
            sample_count = 0;
            start_time = 0;
            initial_depth = 0.0f;
            max_pressure = 0.0f;

            lowpower_profile_peripherals_up();
            bringup_tick = HAL_GetTick();
            state = REC_BRINGUP;
            g_app.mode = SYS_MODE_RECORDING;
            shell_printf("[recorder] Profile peripherals up; waiting %u ms\r\n",
                         PROFILE_BRINGUP_MS);
        }
        break;

    case REC_BRINGUP:
        pb8_discard_start_request();

        if (pb8_low_debounced()) {
            shell_print("[recorder] PB8 LOW during bringup; recording canceled\r\n");
            return_to_idle();
            return;
        }

        if ((HAL_GetTick() - bringup_tick) >= PROFILE_BRINGUP_MS) {
            debounce_active = false;
            g_profile.start_epoch = get_unix_timestamp();

            shell_printf("[recorder] Started (SD powered off, buffering in RAM)\r\n");

            /* Take an immediate probe sample for false-start detection only.
             * Do not append it to g_profile; the first recorded sample is the
             * first scheduled SAMPLE_INTERVAL measurement. */
            sensors_sample(&reading);
            next_sample_tick = HAL_GetTick() + SAMPLE_INTERVAL;
            sample_count = 0;
            start_time = HAL_GetTick();
            initial_depth = reading.ctd.pressure;
            max_pressure = reading.ctd.pressure;

            state = REC_RECORDING;
        }
        break;

    case REC_RECORDING:
        /* Check if PB8 went LOW (debounced) — stop recording, enter normalization */
        if (pb8_low_debounced()) {
            shell_printf("[recorder] PB8 LOW after %u records\r\n", g_profile.count);
            enter_normalization();
            return;
        }

        /* Time for next sample? */
        if ((HAL_GetTick() - next_sample_tick) < 0x80000000UL) {
            record_sample(SAMPLE_INTERVAL);
            sample_count++;

            /* False start detection during validation window */
            if (sample_count <= FALSE_START_SAMPLES) {
                if (reading.ctd.pressure > max_pressure)
                    max_pressure = reading.ctd.pressure;

                if (sample_count == FALSE_START_SAMPLES) {
                    if (max_pressure <= initial_depth + TOLERANCE) {
                        /* No significant descent — discard in-RAM buffer. */
                        g_profile.count = 0;
                        shell_print("[recorder] False start — recording discarded\r\n");
                        return_to_idle();
                        return;
                    }
                    /* Passed false-start check — nothing to record yet; filename is
                     * built at flush time from g_profile.start_epoch. */
                }
            }

            /* Recording timeout — max samples reached, wait for PB8 LOW */
            if (sample_count >= get_max_samples()) {
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
        pb8_discard_start_request();

        if (pb8_low_debounced()) {
            shell_printf("[recorder] PB8 LOW after timeout\r\n");
            enter_normalization();
        }
        break;

    case REC_NORMALIZING:
        /* Ignore PB8 triggers during normalization */
        pb8_discard_start_request();

        if ((HAL_GetTick() - next_sample_tick) < 0x80000000UL) {
            record_sample(NORM_INTERVAL);
            norm_count++;

            if (norm_count >= NORM_SAMPLES) {
                /* Build the realtime response from RAM before touching SD so
                 * the client can still retrieve the profile even if the SD
                 * flush fails. */
                realtime_comm_build(&g_profile);

                /* Done sampling — power SD back up and flush the profile. */
                if (!sd_mount_for_flush()) {
                    shell_print("[recorder] Profile data lost: SD restore failed\r\n");
                    return_to_idle();
                    return;
                }

                if (flush_profile_to_sd()) {
                    strncpy(last_successful_filename, rec_filename,
                            sizeof(last_successful_filename) - 1);
                    last_successful_filename[sizeof(last_successful_filename) - 1] = '\0';
                    shell_printf("[recorder] Complete. %u records in %s\r\n",
                                 g_profile.count, rec_filename);
                } else {
                    shell_print("[recorder] Flush failed\r\n");
                }

                return_to_idle();
            }
        }
        break;
    }
}
