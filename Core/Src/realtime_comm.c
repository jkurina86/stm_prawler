/**
  ******************************************************************************
  * @file    realtime_comm.c
  * @brief   Realtime hex-encoded CSV streaming over WiFi.
 * @note    After each profile finishes normalizing, realtime_comm_build()
 *          formats the CSV payload into rt_buf and caches its frame CRC/length.
 *          realtime_comm_stream() sends "@@@" + CRC + length + "\n", then
 *          the CSV payload bytes.
  ******************************************************************************
  */

#include "realtime_comm.h"
#include "wifi.h"
#include "shell.h"
#include <stdint.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define RT_BUF_SIZE          12288   /* 12 KB: fits worst-case variable-width realtime CSV */
#define RT_LEN_ASCII_LEN     4U

/* Private variables ---------------------------------------------------------*/
static char rt_buf[RT_BUF_SIZE] __attribute__((section(".ram2_bss")));
static uint16_t rt_len;
static uint16_t rt_crc;
/* Flag prevents re-sending the same profile. Initialized to 1 so a
 * realtime request before any profile has been built reports No_Data. */
static uint8_t data_sent = 1;

static uint32_t rt_crc16_xmodem_accum(uint32_t accum, uint8_t ch)
{
    accum |= (uint32_t)ch;

    for (uint8_t bit = 0; bit < 8; bit++) {
        accum <<= 1;
        if (accum & 0x1000000UL) {
            accum ^= 0x102100UL;
        }
    }

    return accum;
}

static uint32_t rt_crc16_xmodem_update(uint32_t accum, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        accum = rt_crc16_xmodem_accum(accum, data[i]);
    }

    return accum;
}

static uint16_t rt_crc16_xmodem_finish(uint32_t accum)
{
    /* CRC-16/XMODEM-compatible finish: init 0x0000, poly 0x1021,
     * two final zero-byte augmentations. Check "123456789" -> 0x31C3. */
    accum = rt_crc16_xmodem_accum(accum, 0U);
    accum = rt_crc16_xmodem_accum(accum, 0U);

    return (uint16_t)(accum >> 8);
}

/* Public functions ----------------------------------------------------------*/

void realtime_comm_build(const profile_data_t *profile)
{
    char *csv_buf = rt_buf;
    char len_ascii[RT_LEN_ASCII_LEN + 1U];
    size_t csv_capacity = RT_BUF_SIZE;
    uint32_t crc_accum = 0;
    uint16_t csv_len;
    uint16_t rows_built = 0;
    bool has_optode = false;
    bool has_wetlab = false;

    switch (profile->sensor_level) {
    case SENSOR_CFG_CTD_ONLY:
        break;
    case SENSOR_CFG_CTD_OPTODE:
        has_optode = true;
        break;
    case SENSOR_CFG_ALL:
    default:
        has_optode = true;
        has_wetlab = true;
        break;
    }

    rt_len = 0;
    rt_crc = 0;
    data_sent = 0;

    /* Create the header based on sensor configuration */
    int n;
    if (has_wetlab && has_optode) {
        n = snprintf(csv_buf, csv_capacity,
                     "EP,CD,CT,CC,OT,O2,CH,TB,CD\n");
    } else if (!has_wetlab && has_optode) {
        n = snprintf(csv_buf, csv_capacity,
                     "EP,CD,CT,CC,OT,O2\n");
    } else {
        n = snprintf(csv_buf, csv_capacity,
                     "EP,CD,CT,CC\n");
    }

    if (n < 0 || (size_t)n >= csv_capacity) {
        return;
    }

    /* Initialize the CSV length */
    csv_len = (uint16_t)n;

    for (uint16_t i = 0; i < profile->count; i++) {
        /* Select the active row */
        const measurement_data_t *m = &profile->measurements[i];

        /* Scale the values */
        int16_t depth = (int16_t)(m->ctd.pressure * 100.0f);
        int16_t temp = (int16_t)(m->ctd.temperature * 1000.0f);
        int16_t cond = (int16_t)(m->ctd.conductivity * 10000.0f);
        uint16_t o2 = (uint16_t)(m->optode.o2_concentration * 100.0f);
        uint16_t o2temp = (uint16_t)(m->optode.temperature * 1000.0f);

        /* Build the new CSV row based on sensor configuration */
        int w;
        if (has_wetlab) {
            w = snprintf(csv_buf + csv_len, csv_capacity - csv_len,
                         "%08lx,%04x,%04x,%04x,%04x,%04x,%04x,%04x,%04x\n",
                         (unsigned long)m->timestamp,
                         (uint16_t)depth,
                         (uint16_t)temp,
                         (uint16_t)cond,
                         o2temp,
                         o2,
                         (uint16_t)m->wetlab.ch1_signal,
                         (uint16_t)m->wetlab.ch2_signal,
                         (uint16_t)m->wetlab.ch3_signal);
        } else if (has_optode) {
            w = snprintf(csv_buf + csv_len, csv_capacity - csv_len,
                         "%08lx,%04x,%04x,%04x,%04x,%04x\n",
                         (unsigned long)m->timestamp,
                         (uint16_t)depth,
                         (uint16_t)temp,
                         (uint16_t)cond,
                         o2temp,
                         o2);
        } else {
            w = snprintf(csv_buf + csv_len, csv_capacity - csv_len,
                         "%08lx,%04x,%04x,%04x\n",
                         (unsigned long)m->timestamp,
                         (uint16_t)depth,
                         (uint16_t)temp,
                         (uint16_t)cond);
        }

        if (w < 0 || (size_t)w >= (csv_capacity - csv_len)) {
            break;
        }

        /* Update CSV length */
        csv_len += (uint16_t)w;
        rows_built++;
    }

    /* Create an ASCII-Encoded hex-length string */
    (void)snprintf(len_ascii, sizeof(len_ascii), "%04X", csv_len);

    /* CRC covers the ASCII length field plus CSV payload bytes only. It does
     * not cover the "@@@" preamble, CRC field, or newline after the length. */
    crc_accum = rt_crc16_xmodem_update(crc_accum, (const uint8_t *)len_ascii, RT_LEN_ASCII_LEN);

    /* Add the CSV data to the CRC. */
    crc_accum = rt_crc16_xmodem_update(crc_accum, (const uint8_t *)rt_buf, csv_len);

    /* Finish the CRC calculation */
    rt_crc = rt_crc16_xmodem_finish(crc_accum);

    /* Update the length of the realtime data transfer */
    rt_len = csv_len;

    shell_printf("[realtime] Built %u rows (%u-byte CSV)\r\n", rows_built, rt_len);
}

void realtime_comm_stream(void)
{
    /* Guard to ensure WiFi is in passthrough mode */
    if (wifi_get_state() != WIFI_STATE_STREAMING) {
        shell_print("[realtime] WiFi not streaming\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }

    /* No_Data case */
    if (data_sent || rt_len == 0) {
        shell_print("[realtime] No_Data\r\n");
        wifi_printf("No_Data\r\n");
        wifi_prompt();
        shell_print(SHELL_PROMPT);
        return;
    }

    /* Sent-byte counter */
    uint16_t sent = 0;

    /* Send the Preamble */
    wifi_printf("@@@%04X%04X\n", rt_crc, rt_len);

    /* Increment the sent-byte counter */
    sent += 3U + 4U + 4U + 1U;

    /* Send the CSV Data */
    sent += wifi_write((const uint8_t *)rt_buf, rt_len);

    /* Set the data_sent flag */
    data_sent = 1;
    wifi_prompt();

    shell_printf("[realtime] Sent %u bytes (%u-byte CSV)\r\n", sent, rt_len);
    shell_print(SHELL_PROMPT);
}
