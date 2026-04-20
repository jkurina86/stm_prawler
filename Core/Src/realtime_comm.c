/**
  ******************************************************************************
  * @file    realtime_comm.c
  * @brief   Realtime hex-encoded CSV streaming over WiFi.
  * @note    After each profile finishes normalizing, realtime_comm_build()
  *          formats the full framed response into rt_buf (placed in SRAM2 via
  *          the .ram2_bss linker section). realtime_comm_stream() then just
  *          writes the pre-built frame to the WiFi TCP client.
  ******************************************************************************
  */

#include "realtime_comm.h"
#include "wifi.h"
#include "shell.h"
#include <stdint.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define RT_BUF_SIZE          18432   /* 18 KB: fits worst-case 346 rows + frame */
#define RT_PREFIX_LEN        3U
#define RT_CRC_ASCII_LEN     4U
#define RT_LEN_ASCII_LEN     4U
#define RT_FRAME_OVERHEAD    (RT_PREFIX_LEN + RT_CRC_ASCII_LEN + RT_LEN_ASCII_LEN)
#define RT_ROW_RESERVE       64U     /* defensive min free bytes before appending a row */

/* Private variables ---------------------------------------------------------*/
static char rt_buf[RT_BUF_SIZE] __attribute__((section(".ram2_bss")));
static uint16_t rt_len;
/* Gate that prevents re-sending the same profile. Initialized to 1 so a
 * realtime request before any profile has been built reports No_Data. */
static uint8_t data_sent = 1;

/* Private helpers -----------------------------------------------------------*/

static void rt_store_hex16(char *dst, uint16_t value)
{
    (void)snprintf(dst, RT_CRC_ASCII_LEN + 1U, "%04X", value);
}

static uint16_t rt_crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U)
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else
                crc <<= 1;
        }
    }

    return crc;
}

/* Public functions ----------------------------------------------------------*/

void realtime_comm_build(const profile_data_t *profile)
{
    char *frame = rt_buf;
    char *csv_buf = &rt_buf[RT_FRAME_OVERHEAD];
    size_t csv_capacity = RT_BUF_SIZE - RT_FRAME_OVERHEAD;
    uint16_t csv_len;
    uint16_t rows_built = 0;
    uint16_t crc;

    rt_len = 0;
    data_sent = 0;

    int n = snprintf(csv_buf, csv_capacity,
                     "datetime,depth,temp,cond,wl_c1,wl_c2,wl_c3,o2,o2temp\r\n");
    if (n < 0 || (size_t)n >= csv_capacity) {
        rt_len = 0;
        shell_print("[realtime] Header format failed\r\n");
        return;
    }
    csv_len = (uint16_t)n;

    for (uint16_t i = 0; i < profile->count; i++) {
        if (csv_capacity - csv_len < RT_ROW_RESERVE) {
            shell_printf("[realtime] Buffer full at row %u\r\n", i);
            break;
        }

        const measurement_data_t *m = &profile->measurements[i];

        int16_t depth = (int16_t)(m->ctd.pressure * 100.0f);
        int16_t temp = (int16_t)(m->ctd.temperature * 1000.0f);
        int16_t cond = (int16_t)(m->ctd.conductivity * 10000.0f);
        uint16_t o2 = (uint16_t)(m->optode.o2_concentration * 100.0f);
        uint16_t o2temp = (uint16_t)(m->optode.temperature * 1000.0f);

        int w = snprintf(csv_buf + csv_len, csv_capacity - csv_len,
                         "%08lx,%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x\r\n",
                         (unsigned long)m->timestamp,
                         (uint16_t)depth,
                         (uint16_t)temp,
                         (uint16_t)cond,
                         (uint16_t)m->wetlab.ch1_signal,
                         (uint16_t)m->wetlab.ch2_signal,
                         (uint16_t)m->wetlab.ch3_signal,
                         o2,
                         o2temp);

        if (w < 0 || (size_t)w >= (csv_capacity - csv_len)) {
            shell_printf("[realtime] Row %u format truncated\r\n", i);
            break;
        }
        csv_len += (uint16_t)w;
        rows_built++;
    }

    frame[0] = '@';
    frame[1] = '@';
    frame[2] = '@';
    rt_store_hex16(&frame[RT_PREFIX_LEN + RT_CRC_ASCII_LEN], csv_len);

    crc = rt_crc16_ccitt_false(
        (const uint8_t *)&frame[RT_PREFIX_LEN + RT_CRC_ASCII_LEN],
        (uint16_t)(RT_LEN_ASCII_LEN + csv_len));
    rt_store_hex16(&frame[RT_PREFIX_LEN], crc);

    rt_len = RT_FRAME_OVERHEAD + csv_len;

    shell_printf("[realtime] Built %u rows (%u-byte CSV, %u-byte frame)\r\n",
                 rows_built, csv_len, rt_len);
}

void realtime_comm_stream(void)
{
    if (wifi_get_state() != WIFI_STATE_STREAMING) {
        shell_print("[realtime] WiFi not streaming\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }

    if (data_sent || rt_len == 0) {
        shell_print("[realtime] No_Data\r\n");
        wifi_printf("No_Data\r\n> ");
        shell_print(SHELL_PROMPT);
        return;
    }

    uint16_t sent = 0;

    sent += wifi_write((const uint8_t *)rt_buf, RT_FRAME_OVERHEAD);
    sent += wifi_write((const uint8_t *)&rt_buf[RT_FRAME_OVERHEAD],
                       (uint16_t)(rt_len - RT_FRAME_OVERHEAD));

    data_sent = 1;
    shell_printf("[realtime] Sent %u/%u bytes\r\n", sent, rt_len);
    shell_print(SHELL_PROMPT);
}
