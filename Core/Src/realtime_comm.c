/**
  ******************************************************************************
  * @file    realtime_comm.c
  * @brief   Realtime CSV streaming over WiFi
  * @note    Reads the last recording CSV from SD, transforms each row into a
  *          compact integer-scaled format, and streams it line-by-line over
  *          the WiFi TCP connection.
  ******************************************************************************
  */

#include "realtime_comm.h"
#include "recorder.h"
#include "filesystem.h"
#include "wifi.h"
#include "shell.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define GPS_TO_UNIX_OFFSET  315964800UL
#define RT_LINE_SIZE        256

/* Private variables ---------------------------------------------------------*/
static char rt_line[RT_LINE_SIZE];

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Skip past n comma-separated fields in a CSV line
  * @param  p Pointer into CSV line
  * @param  n Number of fields to skip
  * @retval Pointer to start of the (n+1)th field, or NULL
  */
static const char *csv_skip(const char *p, int n)
{
    while (n > 0 && *p) {
        if (*p == ',') n--;
        p++;
    }
    return (n == 0) ? p : NULL;
}

/* Public functions ----------------------------------------------------------*/

void realtime_comm_stream(void)
{
    if (wifi_get_state() != WIFI_STATE_STREAMING) {
        shell_print("[realtime] WiFi not streaming\r\n");
        return;
    }

    const char *fname = recorder_get_last_filename();
    if (fname == NULL) {
        shell_print("[realtime] No recording available\r\n");
        wifi_printf("[realtime] No recording available\r\n");
        return;
    }

    shell_printf("[realtime] Streaming %s\r\n", fname);

    if (filesystem_open_read(fname) != FS_OK) {
        shell_print("[realtime] Open failed\r\n");
        return;
    }

    /* Skip source CSV header */
    if (filesystem_readline(rt_line, RT_LINE_SIZE) != FS_OK) {
        shell_print("[realtime] Empty file\r\n");
        filesystem_close_read();
        return;
    }

    /* Send header */
    wifi_printf("datetime,depth,temp,cond,chl,ntu,cdom,o2,o2temp\r\n");

    /* Stream rows line-by-line */
    uint32_t line_count = 0;
    while (filesystem_readline(rt_line, RT_LINE_SIZE) == FS_OK) {
        const char *p = rt_line;

        /* Navigate to needed fields (0-indexed) */
        const char *f1  = csv_skip(p, 1);   /* GPS_Epoch_UTC */
        const char *f2  = csv_skip(p, 2);   /* CTD_C */
        const char *f3  = csv_skip(p, 3);   /* CTD_T */
        const char *f4  = csv_skip(p, 4);   /* CTD_D */
        const char *f5  = csv_skip(p, 5);   /* Optode_O2 */
        const char *f6  = csv_skip(p, 6);   /* Optode_Temp */
        const char *f15 = csv_skip(p, 15);  /* Wetlab_C1_signal */
        const char *f17 = csv_skip(p, 17);  /* Wetlab_C2_signal */
        const char *f19 = csv_skip(p, 19);  /* Wetlab_C3_signal */

        if (!f1 || !f2 || !f3 || !f4 || !f5 || !f6 || !f15 || !f17 || !f19)
            continue;

        uint32_t gps_epoch = strtoul(f1, NULL, 10);
        uint32_t datetime  = gps_epoch + GPS_TO_UNIX_OFFSET;
        int32_t  depth     = (int32_t)(strtof(f4, NULL) * 100);
        int32_t  temp      = (int32_t)(strtof(f3, NULL) * 1000);
        int32_t  cond      = (int32_t)(strtof(f2, NULL) * 10000);
        uint16_t chl       = (uint16_t)strtoul(f15, NULL, 10);
        uint16_t ntu       = (uint16_t)strtoul(f17, NULL, 10);
        uint16_t cdom      = (uint16_t)strtoul(f19, NULL, 10);
        int32_t  o2        = (int32_t)(strtof(f5, NULL) * 100);
        int32_t  o2temp    = (int32_t)(strtof(f6, NULL) * 1000);

        wifi_printf("%lu,%ld,%ld,%ld,%u,%u,%u,%ld,%ld\r\n",
                    datetime, depth, temp, cond,
                    chl, ntu, cdom, o2, o2temp);
        line_count++;
    }

    filesystem_close_read();

    shell_printf("[realtime] Sent %lu lines\r\n", line_count);
}
