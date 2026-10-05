/**
 * @file LI8x0.h
 * @brief LI-COR LI-820/830/840/850 XML driver using STM32 HAL.
 *
 * Adapted from EDD-pCO2Sensor/Firmware/pCO2Sensor/src/LI8x0.c and .h
 * (Matt Casari, NOAA/PMEL/EDD). Protocol: LI-COR publication 977-17191,
 * sections 1 and 2. The application owns sensor power and UART assignment.
 */
#ifndef LI8X0_H
#define LI8X0_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LI8X0_RX_BUF_SIZE       4096U
#define LI8X0_TX_BUF_SIZE        256U
#define LI8X0_TIMEOUT_MS        2000U
#define LI8X0_QUERY_TIMEOUT_MS  5000U
#define LI8X0_REPLY_GAP_MS       100U

/* A single driver instance, consistent with the other sensor drivers. */
typedef enum {
    LI8X0_UNKNOWN = 0,
    LI8X0_LI820 = 820,
    LI8X0_LI830 = 830,
    LI8X0_LI840 = 840,
    LI8X0_LI850 = 850
} li8x0_model_t;

typedef enum {
    LI8X0_OK = 0,
    LI8X0_INVALID_ARG,
    LI8X0_NOT_READY,
    LI8X0_BUSY,
    LI8X0_TIMEOUT,
    LI8X0_UART_ERROR,
    LI8X0_OVERFLOW,
    LI8X0_PARSE_ERROR,
    LI8X0_REJECTED,
    LI8X0_SYNC_REQUIRED
} li8x0_status_t;

enum {
    LI8X0_DATA_CO2        = 1U << 0,
    LI8X0_DATA_CO2ABS     = 1U << 1,
    LI8X0_DATA_H2O        = 1U << 2,
    LI8X0_DATA_H2ODEW     = 1U << 3,
    LI8X0_DATA_H2OABS     = 1U << 4,
    LI8X0_DATA_CELLTEMP   = 1U << 5,
    LI8X0_DATA_CELLPRES   = 1U << 6,
    LI8X0_DATA_IVOLT      = 1U << 7,
    LI8X0_DATA_FLOWRATE   = 1U << 8,
    LI8X0_DATA_RAW_CO2    = 1U << 9,
    LI8X0_DATA_RAW_CO2REF = 1U << 10,
    LI8X0_DATA_RAW_H2O    = 1U << 11,
    LI8X0_DATA_RAW_H2OREF = 1U << 12
};

typedef struct {
    float co2;              /* ppm */
    float co2abs;
    float h2o;              /* mmol/mol, LI-840/850 */
    float h2odewpoint;       /* deg C */
    float h2oabs;
    float celltemp;         /* deg C */
    float cellpres;         /* kPa */
    float ivolt;            /* V */
    float flowrate;
    struct {
        int32_t co2;
        int32_t co2ref;
        int32_t h2o;
        int32_t h2oref;
    } raw;
    uint32_t valid_fields;  /* Missing floats are NAN; raw validity uses this mask. */
} li8x0_data_t;

/* Bit order: CO2 zero/span/span2, then H2O zero/span/span2. */
typedef struct {
    float co2kzero;
    float co2kspan;
    float co2kspan2;
    float h2okzero;
    float h2okspan;
    float h2okspan2;
    char co2lastzero[21];
    char co2lastspan[21];
    char co2lastspan2[21];
    uint32_t valid_fields;
} li8x0_cal_t;

typedef struct {
    li8x0_model_t model;
    bool ack_present;
    bool ack;
    bool data_present;
    bool cal_present;
    char serialnum[32];
    char version[32];
    char error[128];
    li8x0_data_t data;
    li8x0_cal_t cal;
} li8x0_response_t;

/** Bind an exclusively owned UART: 9600 baud, 8N1, normal-mode RX DMA.
 * No peripheral reconfiguration, GPIO writes, discovery or sensor commands.
 * No driver-specific HAL callbacks are needed; normal UART/DMA IRQs remain enabled.
 * Rebinding while a split-phase request is active returns BUSY.
 * Rebinding does not clear an interrupted-transaction synchronization fault.
 */
li8x0_status_t li8x0_init(UART_HandleTypeDef *huart, li8x0_model_t model);
li8x0_model_t li8x0_get_model(void);

/** Discover using the legacy driver's <who> command. Does not power the sensor. */
li8x0_status_t li8x0_autodiscover(void);

/** Configure polled XML output: outrate=0, echo/strip=false, measurement fields on.
 * Leaves heater, pressure compensation, pump, alarms and calibration unchanged.
 * LI-840/850 also enable H2O fields; stops at the first failed ACK.
 */
li8x0_status_t li8x0_configure(void);

li8x0_status_t li8x0_sample(li8x0_data_t *out);
li8x0_status_t li8x0_fire(void);
/** Collect waits for a complete XML reply, bounded by the fire-time deadline.
 * Sampling requires a CO2 field. On failure the caller's output is unchanged.
 */
li8x0_status_t li8x0_collect(li8x0_data_t *out);
void li8x0_cancel(void);

/** Explicitly drain/discard RX after cancellation or a failed transaction.
 * All new commands are blocked with SYNC_REQUIRED until this succeeds.
 * The application must first establish that the previous operation has ended
 * (or reset/power-cycle the sensor); XML has no transaction IDs. settle_ms is
 * an optional 0..5000 ms additional drain interval before requiring 100 ms quiet.
 * Sends no command and never retries calibration. Returns TIMEOUT if RX cannot
 * become quiet within settle_ms + 5000 ms. Does not feed the watchdog.
 */
li8x0_status_t li8x0_resync(uint32_t settle_ms);

/** Query a top-level section (e.g. "cal", "poly", "cfg"), or "" for full state.
 * Data requests have a 2-second deadline; larger state queries have 5 seconds.
 * Parsed data/calibration/identity are returned; other sections remain available
 * as XML through li8x0_last_response(). An ACK alone cannot satisfy a query.
 * Full-state queries require serial number and firmware version in one response.
 */
li8x0_status_t li8x0_query(const char *section, li8x0_response_t *out);
li8x0_status_t li8x0_get_info(li8x0_response_t *out);
li8x0_status_t li8x0_get_cal(li8x0_cal_t *out);

/** Set one scalar XML field, optionally nested: "rs232", "raw", "co2", "TRUE".
 * subfield may be NULL or "". Returns OK only for an affirmative ACK.
 * The sensor validates model-specific settings. Values cannot contain XML markup.
 */
li8x0_status_t li8x0_set(const char *section, const char *field,
                       const char *subfield, const char *value);
li8x0_status_t li8x0_heater(bool enabled);
li8x0_status_t li8x0_set_co2_zero(float value);
li8x0_status_t li8x0_set_co2_span(float value);
li8x0_status_t li8x0_set_co2_span2(float value);

/** Start gas calibration with ISO date yyyy-mm-dd and suitable gas flowing.
 * OK means the command was acknowledged, NOT that calibration has completed.
 * Use get_cal later to read back coefficients/dates. No 60-second busy wait.
 */
li8x0_status_t li8x0_start_zero(const char *date);
li8x0_status_t li8x0_start_span(const char *date, float ppm, bool secondary);

/** Last completed response, driver-owned until the next request/resync. NULL before RX. */
const char *li8x0_last_response(size_t *length);

/** Parse a complete, length-bounded XML document without modifying the input.
 * Tag matching is case-insensitive; string values retain their original case.
 * Malformed XML/numbers and duplicate recognized fields fail atomically.
 * ACK/error responses parse successfully; transaction APIs interpret rejection.
 */
li8x0_status_t li8x0_parse_response(const char *xml, size_t length,
                                  li8x0_response_t *out);
void li8x0_clear_data(li8x0_data_t *data);

#ifdef __cplusplus
}
#endif
#endif /* LI8X0_H */
