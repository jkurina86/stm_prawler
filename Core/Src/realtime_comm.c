/**
  ******************************************************************************
  * @file    realtime_comm.c
  * @brief   Realtime hex-encoded CSV streaming over WiFi.
  * @note    After each profile finishes normalizing, realtime_comm_build()
  *          formats the full response into rt_buf (placed in SRAM2 via the
  *          .ram2_bss linker section). realtime_comm_stream() then just
  *          writes the pre-built buffer to the WiFi TCP client.
  ******************************************************************************
  */

#include "realtime_comm.h"
#include "wifi.h"
#include "shell.h"
#include <stdint.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define RT_BUF_SIZE        18432   /* 18 KB: fits worst-case 346 rows + header */
#define RT_ROW_RESERVE     64      /* defensive min free bytes before appending a row */

/* Private variables ---------------------------------------------------------*/
static char rt_buf[RT_BUF_SIZE] __attribute__((section(".ram2_bss")));
static uint16_t rt_len;
/* Gate that prevents re-sending the same profile. Initialized to 1 so a
 * realtime request before any profile has been built reports No_Data. */
static uint8_t data_sent = 1;

/* Public functions ----------------------------------------------------------*/

void realtime_comm_build(const profile_data_t *profile)
{
    rt_len = 0;
    data_sent = 0;

    int n = snprintf(rt_buf, RT_BUF_SIZE,
                     "datetime,depth,temp,cond,wl_c1,wl_c2,wl_c3,o2,o2temp\r\n");
    if (n < 0 || n >= RT_BUF_SIZE) {
        rt_len = 0;
        shell_print("[realtime] Header format failed\r\n");
        return;
    }
    rt_len = (uint16_t)n;

    for (uint16_t i = 0; i < profile->count; i++) {
        if (RT_BUF_SIZE - rt_len < RT_ROW_RESERVE) {
            shell_printf("[realtime] Buffer full at row %u\r\n", i);
            break;
        }

        const measurement_data_t *m = &profile->measurements[i];

        int16_t depth = (int16_t)(m->ctd.pressure * 100.0f);
        int16_t temp = (int16_t)(m->ctd.temperature * 1000.0f);
        int16_t cond = (int16_t)(m->ctd.conductivity * 10000.0f);
        uint16_t o2 = (uint16_t)(m->optode.o2_concentration * 100.0f);
        uint16_t o2temp = (uint16_t)(m->optode.temperature * 1000.0f);

        int w = snprintf(rt_buf + rt_len, RT_BUF_SIZE - rt_len,
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

        if (w < 0 || (uint16_t)w >= RT_BUF_SIZE - rt_len) {
            shell_printf("[realtime] Row %u format truncated\r\n", i);
            break;
        }
        rt_len += (uint16_t)w;
    }

    shell_printf("[realtime] Built %u rows (%u bytes)\r\n",
                 profile->count, rt_len);
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
        shell_print(SHELL_PROMPT);
        wifi_printf("No_Data\r\n> ");
        return;
    }

    uint16_t sent = wifi_write((const uint8_t *)rt_buf, rt_len);
    data_sent = 1;
    wifi_printf("> ");
    shell_printf("[realtime] Sent %u/%u bytes\r\n", sent, rt_len);
    shell_print(SHELL_PROMPT);
}
