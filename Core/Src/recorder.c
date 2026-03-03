/**
  ******************************************************************************
  * @file    recorder.c
  * @brief   PB8-triggered continuous sensor logging to SD card
  * @note    When PB8 goes HIGH, samples all 3 sensors every 4s and writes
  *          binary records to a double-buffered log file. When PB8 goes LOW,
  *          flushes remaining data and closes the file.
  ******************************************************************************
  */

#include "recorder.h"
#include "main.h"
#include "shell.h"
#include "filesystem.h"
#include "ab-rtcmc-rtc.h"
#include "ctd.h"
#include "optode.h"
#include "wetlab.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define REC_BUF_SIZE      4096
#define SAMPLE_INTERVAL   4000  /* ms between samples */

/* Private types -------------------------------------------------------------*/
typedef enum {
    REC_IDLE,
    REC_RECORDING
} rec_state_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t rec_buf[2][REC_BUF_SIZE];
static uint8_t active_buf;
static uint16_t buf_offset;

static FIL rec_file;
static rec_state_t state;
static uint32_t record_num;
static uint32_t next_sample_tick;
static uint16_t file_counter;
static char rec_filename[16];

/* start_flag set by EXTI ISR on PB8 rising edge (owned by main.c) */
extern volatile bool start_flag;

/* GPS epoch helpers ---------------------------------------------------------*/

static bool is_leap_year(uint16_t year)
{
    return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}

static uint32_t rtc_to_gps_epoch(const RTC_DateTime_t *dt)
{
    /* GPS epoch: Jan 6, 1980 00:00:00 UTC */
    static const uint16_t days_before_month[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };

    uint16_t year = 2000 + dt->years;

    /* Days from Jan 1 1980 to Jan 1 of target year */
    uint32_t days = 0;
    for (uint16_t y = 1980; y < year; y++) {
        days += is_leap_year(y) ? 366 : 365;
    }

    /* Add days for completed months in target year */
    if (dt->months >= 1 && dt->months <= 12) {
        days += days_before_month[dt->months - 1];
    }

    /* Add leap day if past Feb in a leap year */
    if (dt->months > 2 && is_leap_year(year)) {
        days += 1;
    }

    /* Add day of month (1-based) */
    days += dt->days - 1;

    /* Subtract 5 days to shift from Jan 1 to Jan 6 */
    days -= 5;

    return days * 86400UL + dt->hours * 3600UL + dt->minutes * 60UL + dt->seconds;
}

/* Private functions ---------------------------------------------------------*/

static uint32_t get_timestamp(void)
{
    RTC_DateTime_t dt = {0};
    if (RTC_GetDateTime(&dt) != RTC_OK) {
        return 0;
    }
    return rtc_to_gps_epoch(&dt);
}

static bool flush_buffer(const uint8_t *buf, uint16_t len)
{
    if (len == 0)
        return true;

    UINT bw;
    FRESULT fr = f_write(&rec_file, buf, len, &bw);
    if (fr != FR_OK || bw != len) {
        shell_printf("[recorder] Write error (fr=%d, wrote %u/%u)\r\n", fr, bw, len);
        return false;
    }

    fr = f_sync(&rec_file);
    if (fr != FR_OK) {
        shell_printf("[recorder] Sync error (fr=%d)\r\n", fr);
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

    /* Find an unused filename: rec_0001.bin, rec_0002.bin, ... */
    for (;;) {
        file_counter++;
        if (file_counter > 9999) {
            shell_print("[recorder] No available filenames\r\n");
            return false;
        }

        snprintf(rec_filename, sizeof(rec_filename), "rec_%04u.bin", file_counter);

        FRESULT fr = f_open(&rec_file, rec_filename, FA_WRITE | FA_CREATE_NEW);
        if (fr == FR_OK) {
            return true;
        }
        if (fr != FR_EXIST) {
            shell_printf("[recorder] Open failed (fr=%d)\r\n", fr);
            return false;
        }
        /* FR_EXIST — try next number */
    }
}

static void sample_sensors(void)
{
    ctd_data_t ctd = {0};
    optode_data_t optode = {0};
    wetlab_data_t wetlab = {0};

    /* Same sequence as handle_sensors */
    uint32_t t0 = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  /* WetLab power on */

    optode_wake();
    ctd_wakeup();

    ctd_fire();
    optode_fire();

    while ((HAL_GetTick() - t0) < 1500)
        ;

    wetlab_fire();

    while ((HAL_GetTick() - t0) < 3500)
        ;

    ctd_collect(&ctd);
    optode_collect(&optode);
    wetlab_collect(&wetlab);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  /* WetLab power off */

    /* Build record */
    record_data_t rec;
    rec.magic = RECORD_MAGIC;
    rec.record_num = record_num;
    rec.timestamp = get_timestamp();
    rec.ctd = ctd;
    rec.optode = optode;
    rec.wetlab = wetlab;

    /* Check if record fits in current buffer */
    if (buf_offset + sizeof(record_data_t) > REC_BUF_SIZE) {
        flush_buffer(rec_buf[active_buf], buf_offset);
        active_buf ^= 1;
        buf_offset = 0;
    }

    memcpy(&rec_buf[active_buf][buf_offset], &rec, sizeof(record_data_t));
    buf_offset += sizeof(record_data_t);
    record_num++;
}

/* Public functions ----------------------------------------------------------*/

void recorder_init(void)
{
    state = REC_IDLE;
    active_buf = 0;
    buf_offset = 0;
    record_num = 0;
    file_counter = 0;
}

void recorder_tick(void)
{
    switch (state) {

    case REC_IDLE:
        if (start_flag) {
            start_flag = false;

            if (!open_log_file()) {
                return;
            }

            record_num = 0;
            buf_offset = 0;
            active_buf = 0;

            shell_printf("[recorder] Started: %s\r\n", rec_filename);

            /* Take first sample immediately */
            sample_sensors();
            next_sample_tick = HAL_GetTick() + SAMPLE_INTERVAL;

            state = REC_RECORDING;
        }
        break;

    case REC_RECORDING:
        /* Check if PB8 went LOW — stop recording */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET) {
            /* Flush remaining data */
            flush_buffer(rec_buf[active_buf], buf_offset);
            f_close(&rec_file);

            shell_printf("[recorder] Stopped. %lu records saved to %s\r\n",
                         record_num, rec_filename);

            state = REC_IDLE;
            return;
        }

        /* Time for next sample? */
        if ((HAL_GetTick() - next_sample_tick) < 0x80000000UL) {
            sample_sensors();
            next_sample_tick += SAMPLE_INTERVAL;
        }
        break;
    }
}
