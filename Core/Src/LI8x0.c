/**
 * @file LI8x0.c
 * @brief STM32 HAL adaptation of the NOAA/PMEL/EDD LI-8x0 driver.
 *
 * Reference: EDD-pCO2Sensor/Firmware/pCO2Sensor/src/LI8x0.c (Matt Casari).
 * XML commands/units: LI-COR 977-17191, sections 1-7 to 1-13 and 2-1 to 2-13.
 */
#include "LI8x0.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define XML_NAME_SIZE 24U
#define XML_MAX_DEPTH 6U

typedef struct {
    char name[XML_NAME_SIZE];
    const char *value;
    size_t length;
    const char *next;
    bool children;
} xml_element_t;

typedef struct {
    const char *name;
    size_t offset;
    uint32_t bit;
    bool integer;
} numeric_field_t;

static UART_HandleTypeDef *li8x0_huart;
static li8x0_model_t li8x0_model;
static uint8_t rx_buf[LI8X0_RX_BUF_SIZE];
static char tx_buf[LI8X0_TX_BUF_SIZE];
static bool pending;
static bool needs_resync;
static uint32_t request_started;
static uint32_t request_timeout;
static size_t last_start, last_length;

static const numeric_field_t data_fields[] = {
    {"co2", offsetof(li8x0_data_t, co2), LI8X0_DATA_CO2, false},
    {"co2abs", offsetof(li8x0_data_t, co2abs), LI8X0_DATA_CO2ABS, false},
    {"h2o", offsetof(li8x0_data_t, h2o), LI8X0_DATA_H2O, false},
    {"h2odewpoint", offsetof(li8x0_data_t, h2odewpoint), LI8X0_DATA_H2ODEW, false},
    {"h2oabs", offsetof(li8x0_data_t, h2oabs), LI8X0_DATA_H2OABS, false},
    {"celltemp", offsetof(li8x0_data_t, celltemp), LI8X0_DATA_CELLTEMP, false},
    {"cellpres", offsetof(li8x0_data_t, cellpres), LI8X0_DATA_CELLPRES, false},
    {"ivolt", offsetof(li8x0_data_t, ivolt), LI8X0_DATA_IVOLT, false},
    {"flowrate", offsetof(li8x0_data_t, flowrate), LI8X0_DATA_FLOWRATE, false}
};
static const numeric_field_t raw_fields[] = {
    {"co2", offsetof(li8x0_data_t, raw.co2), LI8X0_DATA_RAW_CO2, true},
    {"co2ref", offsetof(li8x0_data_t, raw.co2ref), LI8X0_DATA_RAW_CO2REF, true},
    {"h2o", offsetof(li8x0_data_t, raw.h2o), LI8X0_DATA_RAW_H2O, true},
    {"h2oref", offsetof(li8x0_data_t, raw.h2oref), LI8X0_DATA_RAW_H2OREF, true}
};
static const numeric_field_t cal_fields[] = {
    {"co2kzero", offsetof(li8x0_cal_t, co2kzero), 1U << 0, false},
    {"co2kspan", offsetof(li8x0_cal_t, co2kspan), 1U << 1, false},
    {"co2kspan2", offsetof(li8x0_cal_t, co2kspan2), 1U << 2, false},
    {"h2okzero", offsetof(li8x0_cal_t, h2okzero), 1U << 3, false},
    {"h2okspan", offsetof(li8x0_cal_t, h2okspan), 1U << 4, false},
    {"h2okspan2", offsetof(li8x0_cal_t, h2okspan2), 1U << 5, false}
};

static const char *model_name(li8x0_model_t model)
{
    switch (model) {
    case LI8X0_LI820: return "li820";
    case LI8X0_LI830: return "li830";
    case LI8X0_LI840: return "li840";
    case LI8X0_LI850: return "li850";
    default: return NULL;
    }
}

static bool name_equal(const char *a, const char *b)
{
    while (*a && *b) {
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return false;
    }
    return *a == *b;
}

static li8x0_model_t name_model(const char *name)
{
    static const li8x0_model_t models[] = {
        LI8X0_LI820, LI8X0_LI830, LI8X0_LI840, LI8X0_LI850
    };
    for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); i++)
        if (strcmp(name, model_name(models[i])) == 0)
            return models[i];
    return LI8X0_UNKNOWN;
}

static const char *skip_space(const char *p, const char *end)
{
    while (p < end && isspace((unsigned char)*p)) p++;
    return p;
}

static bool read_name(const char **cursor, const char *end, char *name)
{
    size_t n = 0;
    while (*cursor < end &&
           (isalnum((unsigned char)**cursor) || **cursor == '_')) {
        if (n >= XML_NAME_SIZE - 1) return false;
        name[n++] = (char)tolower((unsigned char)*(*cursor)++);
    }
    name[n] = '\0';
    return n > 0;
}

/* Validate balanced, bounded, attribute-free LI-COR XML, not general XML. */
static bool read_element(const char *p, const char *end, unsigned depth,
                         xml_element_t *element)
{
    if (depth >= XML_MAX_DEPTH || p >= end || *p++ != '<') return false;
    if (!read_name(&p, end, element->name)) return false;
    p = skip_space(p, end);
    element->children = false;
    if (p < end && *p == '/') {
        p++;
        if (p >= end || *p++ != '>') return false;
        element->value = p;
        element->length = 0;
        element->next = p;
        return true;
    }
    if (p >= end || *p++ != '>') return false;
    element->value = p;
    const char *text = p;
    while (p < end) {
        if (*p != '<') { p++; continue; }
        if (p + 1 < end && p[1] == '/') {
            char closing[XML_NAME_SIZE];
            const char *close = p + 2;
            if (!read_name(&close, end, closing)) return false;
            close = skip_space(close, end);
            if (close >= end || *close++ != '>' ||
                strcmp(closing, element->name) != 0) return false;
            if (element->children && skip_space(text, p) != p) return false;
            element->length = (size_t)(p - element->value);
            element->next = close;
            return true;
        }
        if (skip_space(text, p) != p) return false;
        xml_element_t child;
        if (!read_element(p, end, depth + 1, &child)) return false;
        element->children = true;
        p = text = child.next;
    }
    return false;
}

/* 1 = found, 0 = absent, -1 = invalid/duplicate. Parent is already validated. */
static int child_element(const xml_element_t *parent, const char *name,
                         xml_element_t *out)
{
    if (!parent->children) return 0;
    const char *end = parent->value + parent->length;
    const char *p = skip_space(parent->value, end);
    int found = 0;
    while (p < end) {
        xml_element_t child;
        if (!read_element(p, end, 0, &child)) return -1;
        if (name_equal(child.name, name)) {
            if (found) return -1;
            *out = child;
            found = 1;
        }
        p = skip_space(child.next, end);
    }
    return found;
}

static bool copy_text(const xml_element_t *element, char *out, size_t capacity)
{
    if (element->children) return false;
    const char *end = element->value + element->length;
    const char *start = skip_space(element->value, end);
    while (end > start && isspace((unsigned char)end[-1])) end--;
    size_t length = (size_t)(end - start);
    if (length >= capacity || memchr(start, '\0', length)) return false;
    memcpy(out, start, length);
    out[length] = '\0';
    return true;
}

static bool read_string(const xml_element_t *parent, const char *name,
                        char *out, size_t capacity)
{
    xml_element_t element;
    int result = child_element(parent, name, &element);
    return result == 0 || (result == 1 && copy_text(&element, out, capacity));
}

static bool read_numbers(const xml_element_t *parent, void *out, uint32_t *mask,
                         const numeric_field_t *fields, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        xml_element_t element;
        int found = child_element(parent, fields[i].name, &element);
        if (found < 0) return false;
        if (!found) continue;
        char text[48], *end;
        if (!copy_text(&element, text, sizeof(text)) || !text[0]) return false;
        errno = 0;
        if (fields[i].integer) {
            long value = strtol(text, &end, 10);
            if (errno || end == text || *end || value < INT32_MIN || value > INT32_MAX)
                return false;
            int32_t raw = (int32_t)value;
            memcpy((char *)out + fields[i].offset, &raw, sizeof(raw));
        } else {
            if (strspn(text, "0123456789.+-eE") != strlen(text)) return false;
            float value = strtof(text, &end);
            if (errno || end == text || *end || !isfinite(value)) return false;
            memcpy((char *)out + fields[i].offset, &value, sizeof(value));
        }
        *mask |= fields[i].bit;
    }
    return true;
}

void li8x0_clear_data(li8x0_data_t *data)
{
    if (!data) return;
    memset(data, 0, sizeof(*data));
    for (size_t i = 0; i < sizeof(data_fields) / sizeof(data_fields[0]); i++) {
        float missing = NAN;
        memcpy((char *)data + data_fields[i].offset, &missing, sizeof(missing));
    }
}

li8x0_status_t li8x0_parse_response(const char *xml, size_t length,
                                  li8x0_response_t *out)
{
    if (!xml || !out) return LI8X0_INVALID_ARG;
    if (!length || length >= LI8X0_RX_BUF_SIZE) return LI8X0_PARSE_ERROR;
    xml_element_t root, element;
    const char *end = xml + length;
    if (!read_element(skip_space(xml, end), end, 0, &root) ||
        skip_space(root.next, end) != end) return LI8X0_PARSE_ERROR;
    li8x0_response_t response = {0};
    response.model = name_model(root.name);
    if (response.model == LI8X0_UNKNOWN) return LI8X0_PARSE_ERROR;
    li8x0_clear_data(&response.data);
    for (size_t i = 0; i < sizeof(cal_fields) / sizeof(cal_fields[0]); i++) {
        float missing = NAN;
        memcpy((char *)&response.cal + cal_fields[i].offset, &missing, sizeof(missing));
    }
    int found = child_element(&root, "ack", &element);
    if (found < 0) return LI8X0_PARSE_ERROR;
    if (found) {
        char value[8];
        if (!copy_text(&element, value, sizeof(value))) return LI8X0_PARSE_ERROR;
        for (size_t i = 0; value[i]; i++) value[i] = (char)tolower((unsigned char)value[i]);
        if (strcmp(value, "true") && strcmp(value, "false")) return LI8X0_PARSE_ERROR;
        response.ack_present = true;
        response.ack = strcmp(value, "true") == 0;
    }
    if (!read_string(&root, "serialnum", response.serialnum, sizeof(response.serialnum)) ||
        !read_string(&root, "ver", response.version, sizeof(response.version)) ||
        !read_string(&root, "error", response.error, sizeof(response.error)))
        return LI8X0_PARSE_ERROR;
    found = child_element(&root, "data", &element);
    if (found < 0) return LI8X0_PARSE_ERROR;
    if (found) {
        response.data_present = true;
        if (!read_numbers(&element, &response.data, &response.data.valid_fields,
                          data_fields, sizeof(data_fields) / sizeof(data_fields[0])))
            return LI8X0_PARSE_ERROR;
        xml_element_t raw;
        int has_raw = child_element(&element, "raw", &raw);
        if (has_raw < 0 || (has_raw &&
            !read_numbers(&raw, &response.data, &response.data.valid_fields,
                          raw_fields, sizeof(raw_fields) / sizeof(raw_fields[0]))))
            return LI8X0_PARSE_ERROR;
    }
    found = child_element(&root, "cal", &element);
    if (found < 0) return LI8X0_PARSE_ERROR;
    if (found) {
        response.cal_present = true;
        if (!read_numbers(&element, &response.cal, &response.cal.valid_fields,
                          cal_fields, sizeof(cal_fields) / sizeof(cal_fields[0])) ||
            !read_string(&element, "co2lastzero", response.cal.co2lastzero,
                         sizeof(response.cal.co2lastzero)) ||
            !read_string(&element, "co2lastspan", response.cal.co2lastspan,
                         sizeof(response.cal.co2lastspan)) ||
            !read_string(&element, "co2lastspan2", response.cal.co2lastspan2,
                         sizeof(response.cal.co2lastspan2))) return LI8X0_PARSE_ERROR;
    }
    *out = response;
    return LI8X0_OK;
}

static bool uart_valid(const UART_HandleTypeDef *huart)
{
    return huart && huart->hdmarx && huart->hdmarx->Init.Mode == DMA_NORMAL &&
        huart->hdmarx->Init.Direction == DMA_PERIPH_TO_MEMORY &&
        huart->hdmarx->Init.PeriphInc == DMA_PINC_DISABLE &&
        huart->hdmarx->Init.MemInc == DMA_MINC_ENABLE &&
        huart->hdmarx->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
        huart->hdmarx->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
        huart->Init.BaudRate == 9600 && huart->Init.WordLength == UART_WORDLENGTH_8B &&
        huart->Init.StopBits == UART_STOPBITS_1 && huart->Init.Parity == UART_PARITY_NONE &&
        huart->Init.Mode == UART_MODE_TX_RX && huart->Init.HwFlowCtl == UART_HWCONTROL_NONE;
}

li8x0_status_t li8x0_init(UART_HandleTypeDef *huart, li8x0_model_t model)
{
    if (pending) return LI8X0_BUSY;
    if (!uart_valid(huart) ||
        (model != LI8X0_UNKNOWN && !model_name(model))) return LI8X0_INVALID_ARG;
    li8x0_huart = huart;
    li8x0_model = model;
    last_length = 0;
    return LI8X0_OK;
}

li8x0_model_t li8x0_get_model(void) { return li8x0_model; }

static li8x0_status_t finish(li8x0_status_t status)
{
    if (pending) {
        HAL_StatusTypeDef result = HAL_UART_AbortReceive(li8x0_huart);
        if (status == LI8X0_OK && result != HAL_OK) status = LI8X0_UART_ERROR;
        if (status != LI8X0_OK) needs_resync = true;
        pending = false;
    }
    if (last_length) rx_buf[last_start + last_length] = '\0';
    return status;
}

void li8x0_cancel(void)
{
    if (pending) needs_resync = true;
    (void)finish(LI8X0_OK);
}

li8x0_status_t li8x0_resync(uint32_t settle_ms)
{
    if (pending) return LI8X0_BUSY;
    if (!li8x0_huart) return LI8X0_NOT_READY;
    if (!uart_valid(li8x0_huart) || settle_ms > LI8X0_QUERY_TIMEOUT_MS)
        return LI8X0_INVALID_ARG;
    needs_resync = true;
    last_length = last_start = 0;
    __HAL_UART_CLEAR_OREFLAG(li8x0_huart);
    __HAL_UART_CLEAR_NEFLAG(li8x0_huart);
    __HAL_UART_CLEAR_FEFLAG(li8x0_huart);
    __HAL_UART_CLEAR_PEFLAG(li8x0_huart);
    __HAL_UART_SEND_REQ(li8x0_huart, UART_RXDATA_FLUSH_REQUEST);
    HAL_StatusTypeDef result = HAL_UART_Receive_DMA(li8x0_huart, rx_buf, sizeof(rx_buf) - 1);
    if (result != HAL_OK) return result == HAL_BUSY ? LI8X0_BUSY : LI8X0_UART_ERROR;
    __HAL_DMA_DISABLE_IT(li8x0_huart->hdmarx, DMA_IT_HT);
    pending = true;
    uint32_t started = HAL_GetTick(), changed = started;
    size_t previous_total = 0;
    while (1) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - started) >= settle_ms + LI8X0_QUERY_TIMEOUT_MS)
            return finish(LI8X0_TIMEOUT);
        if (li8x0_huart->ErrorCode != HAL_UART_ERROR_NONE)
            return finish(LI8X0_UART_ERROR);
        size_t remaining = __HAL_DMA_GET_COUNTER(li8x0_huart->hdmarx);
        if (remaining > sizeof(rx_buf) - 1) return finish(LI8X0_UART_ERROR);
        size_t total = sizeof(rx_buf) - 1 - remaining;
        if (total != previous_total) { changed = now; previous_total = total; }
        if (total == sizeof(rx_buf) - 1) return finish(LI8X0_OVERFLOW);
        if ((uint32_t)(now - started) >= settle_ms &&
            (uint32_t)(now - changed) >= LI8X0_REPLY_GAP_MS) {
            li8x0_status_t status = finish(LI8X0_OK);
            if (status == LI8X0_OK) needs_resync = false;
            return status;
        }
    }
}

const char *li8x0_last_response(size_t *length)
{
    if (length) *length = last_length;
    return last_length ? (const char *)rx_buf + last_start : NULL;
}

static li8x0_status_t begin(bool discovery, uint32_t timeout)
{
    if (pending) return LI8X0_BUSY;
    if (!li8x0_huart || (!discovery && !model_name(li8x0_model))) return LI8X0_NOT_READY;
    if (!uart_valid(li8x0_huart)) return LI8X0_INVALID_ARG;
    if (needs_resync) return LI8X0_SYNC_REQUIRED;
    last_length = last_start = 0;
    __HAL_UART_CLEAR_OREFLAG(li8x0_huart);
    __HAL_UART_CLEAR_NEFLAG(li8x0_huart);
    __HAL_UART_CLEAR_FEFLAG(li8x0_huart);
    __HAL_UART_CLEAR_PEFLAG(li8x0_huart);
    __HAL_UART_SEND_REQ(li8x0_huart, UART_RXDATA_FLUSH_REQUEST);
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(li8x0_huart, rx_buf, sizeof(rx_buf) - 1);
    if (status != HAL_OK) return status == HAL_BUSY ? LI8X0_BUSY : LI8X0_UART_ERROR;
    __HAL_DMA_DISABLE_IT(li8x0_huart->hdmarx, DMA_IT_HT);
    pending = true;
    request_timeout = timeout;
    request_started = HAL_GetTick();
    status = HAL_UART_Transmit(li8x0_huart, (const uint8_t *)tx_buf,
                               (uint16_t)strlen(tx_buf), request_timeout);
    if (status != HAL_OK)
        return finish(status == HAL_TIMEOUT ? LI8X0_TIMEOUT : LI8X0_UART_ERROR);
    return LI8X0_OK;
}

static bool response_has(const char *xml, size_t length, const char *section,
                         const li8x0_response_t *response)
{
    if (!section[0])
        return response->serialnum[0] && response->version[0];
    xml_element_t root, element;
    const char *end = xml + length;
    return read_element(skip_space(xml, end), end, 0, &root) &&
           child_element(&root, section, &element) == 1;
}

/* DMA runs continuously across UART idle gaps. Complete root tags delimit frames.
 * ACK-only frames do not satisfy queries. Drain a trailing ACK or a quiet gap
 * before releasing the UART, so it cannot acknowledge the next setting instead.
 */
static li8x0_status_t receive(const char *section, bool setting, li8x0_response_t *out)
{
    if (!pending) return LI8X0_NOT_READY;
    size_t consumed = 0, scan = 0, previous_total = 0;
    uint32_t changed = HAL_GetTick();
    bool got_payload = false;
    li8x0_response_t payload = {0};
    while (1) {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - request_started) >= request_timeout)
            return finish(LI8X0_TIMEOUT);
        if (li8x0_huart->ErrorCode != HAL_UART_ERROR_NONE)
            return finish(LI8X0_UART_ERROR);
        size_t remaining = __HAL_DMA_GET_COUNTER(li8x0_huart->hdmarx);
        if (remaining > sizeof(rx_buf) - 1) return finish(LI8X0_UART_ERROR);
        size_t total = sizeof(rx_buf) - 1 - remaining;
        __DMB();
        if (total != previous_total) { changed = now; previous_total = total; }
        while (scan + 8 <= total) {
            if (rx_buf[scan] != '<' || rx_buf[scan + 1] != '/') { scan++; continue; }
            char name[6];
            for (size_t i = 0; i < 5; i++) name[i] = (char)tolower(rx_buf[scan + 2 + i]);
            name[5] = '\0';
            if (rx_buf[scan + 7] != '>' || name_model(name) == LI8X0_UNKNOWN) {
                scan++; continue;
            }
            size_t frame_end = scan + 8;
            const char *frame = (const char *)rx_buf + consumed;
            size_t frame_length = frame_end - consumed;
            li8x0_response_t response;
            /* Echoed commands contain '?' or values, not a response payload. */
            size_t command_length = strlen(tx_buf) - 2;
            const char *trimmed = skip_space(frame, frame + frame_length);
            bool echo = (size_t)(frame + frame_length - trimmed) == command_length &&
                        memcmp(trimmed, tx_buf, command_length) == 0;
            li8x0_status_t status = echo ? LI8X0_OK :
                li8x0_parse_response(frame, frame_length, &response);
            if (!echo) {
                if (status != LI8X0_OK) return finish(status);
                if (response.model != li8x0_model) return finish(LI8X0_REJECTED);
                if (response.error[0] || (response.ack_present && !response.ack)) {
                    last_start = consumed; last_length = frame_length;
                    return finish(LI8X0_REJECTED);
                }
                if (setting && response.ack_present && response.ack) {
                    last_start = consumed; last_length = frame_length;
                    return finish(LI8X0_OK);
                }
                if (!setting && response_has(frame, frame_length, section, &response)) {
                    if (name_equal(section, "data") &&
                        !(response.data.valid_fields & LI8X0_DATA_CO2))
                        return finish(LI8X0_PARSE_ERROR);
                    payload = response;
                    got_payload = true;
                    last_start = consumed; last_length = frame_length;
                } else if (got_payload && response.ack_present && response.ack) {
                    status = finish(LI8X0_OK);
                    if (status == LI8X0_OK) *out = payload;
                    return status;
                }
            }
            scan = consumed = frame_end;
        }
        /* Consume inter-frame whitespace without mistaking a partial ACK for quiet. */
        if (got_payload && skip_space((const char *)rx_buf + consumed,
                                       (const char *)rx_buf + total) == (const char *)rx_buf + total &&
            (uint32_t)(now - changed) >= LI8X0_REPLY_GAP_MS) {
            li8x0_status_t status = finish(LI8X0_OK);
            if (status == LI8X0_OK) *out = payload;
            return status;
        }
        if (total == sizeof(rx_buf) - 1) return finish(LI8X0_OVERFLOW);
    }
}

static bool valid_name(const char *name)
{
    if (!name || !name[0]) return false;
    size_t n = 0;
    while (name[n]) {
        if (n >= XML_NAME_SIZE - 1 ||
            !(isalnum((unsigned char)name[n]) || name[n] == '_')) return false;
        n++;
    }
    return true;
}

static bool valid_value(const char *value)
{
    if (!value || !value[0]) return false;
    for (size_t i = 0; value[i]; i++) {
        if (i >= 64 || !(isalnum((unsigned char)value[i]) ||
            strchr(".+-?_: ", value[i]))) return false;
    }
    return true;
}

static li8x0_status_t command_size(int length)
{
    return length < 0 || (size_t)length >= sizeof(tx_buf) ? LI8X0_OVERFLOW : LI8X0_OK;
}

static li8x0_status_t query_begin(const char *section)
{
    if (pending) return LI8X0_BUSY;
    if (!section || (section[0] && !valid_name(section))) return LI8X0_INVALID_ARG;
    const char *root = model_name(li8x0_model);
    if (!root) return LI8X0_NOT_READY;
    int n = section[0] ?
        snprintf(tx_buf, sizeof(tx_buf), "<%s><%s>?</%s></%s>\r\n", root, section, section, root) :
        snprintf(tx_buf, sizeof(tx_buf), "<%s>?</%s>\r\n", root, root);
    li8x0_status_t status = command_size(n);
    uint32_t timeout = name_equal(section, "data") ? LI8X0_TIMEOUT_MS : LI8X0_QUERY_TIMEOUT_MS;
    return status == LI8X0_OK ? begin(false, timeout) : status;
}

li8x0_status_t li8x0_fire(void) { return query_begin("data"); }

li8x0_status_t li8x0_collect(li8x0_data_t *out)
{
    if (!out) return LI8X0_INVALID_ARG;
    li8x0_response_t response;
    li8x0_status_t status = receive("data", false, &response);
    if (status == LI8X0_OK) *out = response.data;
    return status;
}

li8x0_status_t li8x0_sample(li8x0_data_t *out)
{
    if (!out) return LI8X0_INVALID_ARG;
    li8x0_status_t status = li8x0_fire();
    return status == LI8X0_OK ? li8x0_collect(out) : status;
}

li8x0_status_t li8x0_query(const char *section, li8x0_response_t *out)
{
    if (!out) return LI8X0_INVALID_ARG;
    li8x0_status_t status = query_begin(section);
    return status == LI8X0_OK ? receive(section, false, out) : status;
}

li8x0_status_t li8x0_get_info(li8x0_response_t *out)
{
    if (!out) return LI8X0_INVALID_ARG;
    li8x0_response_t response;
    li8x0_status_t status = li8x0_query("", &response);
    if (status != LI8X0_OK) return status;
    if (!response.serialnum[0] || !response.version[0]) return LI8X0_PARSE_ERROR;
    *out = response;
    return LI8X0_OK;
}

li8x0_status_t li8x0_get_cal(li8x0_cal_t *out)
{
    if (!out) return LI8X0_INVALID_ARG;
    li8x0_response_t response;
    li8x0_status_t status = li8x0_query("cal", &response);
    if (status != LI8X0_OK) return status;
    if (!response.cal.valid_fields) return LI8X0_PARSE_ERROR;
    *out = response.cal;
    return LI8X0_OK;
}

li8x0_status_t li8x0_set(const char *section, const char *field,
                       const char *subfield, const char *value)
{
    if (pending) return LI8X0_BUSY;
    if (!valid_name(section) || !valid_name(field) || !valid_value(value) ||
        (subfield && subfield[0] && !valid_name(subfield))) return LI8X0_INVALID_ARG;
    const char *root = model_name(li8x0_model);
    if (!root) return LI8X0_NOT_READY;
    int n;
    if (subfield && subfield[0])
        n = snprintf(tx_buf, sizeof(tx_buf), "<%s><%s><%s><%s>%s</%s></%s></%s></%s>\r\n",
                     root, section, field, subfield, value, subfield, field, section, root);
    else
        n = snprintf(tx_buf, sizeof(tx_buf), "<%s><%s><%s>%s</%s></%s></%s>\r\n",
                     root, section, field, value, field, section, root);
    li8x0_status_t status = command_size(n);
    if (status == LI8X0_OK) status = begin(false, LI8X0_TIMEOUT_MS);
    return status == LI8X0_OK ? receive("", true, NULL) : status;
}

li8x0_status_t li8x0_heater(bool enabled)
{
    return li8x0_set("cfg", "heater", NULL, enabled ? "TRUE" : "FALSE");
}

static li8x0_status_t set_coefficient(const char *field, float value)
{
    if (!isfinite(value)) return LI8X0_INVALID_ARG;
    char text[48];
    int n = snprintf(text, sizeof(text), "%.8g", (double)value);
    if (n < 0 || (size_t)n >= sizeof(text)) return LI8X0_OVERFLOW;
    return li8x0_set("cal", field, NULL, text);
}

li8x0_status_t li8x0_set_co2_zero(float value) { return set_coefficient("co2kzero", value); }
li8x0_status_t li8x0_set_co2_span(float value) { return set_coefficient("co2kspan", value); }
li8x0_status_t li8x0_set_co2_span2(float value) { return set_coefficient("co2kspan2", value); }

li8x0_status_t li8x0_configure(void)
{
    li8x0_status_t status = li8x0_set("cfg", "outrate", NULL, "0");
    if (status != LI8X0_OK) return status;
    static const struct { const char *field, *value; } fields[] = {
        {"echo", "FALSE"}, {"strip", "FALSE"}, {"co2", "TRUE"},
        {"celltemp", "TRUE"}, {"cellpres", "TRUE"}, {"ivolt", "TRUE"},
        {"co2abs", "FALSE"}, {"raw", "TRUE"}
    };
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        status = li8x0_set("rs232", fields[i].field, NULL, fields[i].value);
        if (status != LI8X0_OK) return status;
    }
    if (li8x0_model == LI8X0_LI830 || li8x0_model == LI8X0_LI850) {
        status = li8x0_set("rs232", "flowrate", NULL, "FALSE");
        if (status != LI8X0_OK) return status;
    }
    if (li8x0_model == LI8X0_LI840 || li8x0_model == LI8X0_LI850) {
        static const char *water[] = {"h2o", "h2oabs", "h2odewpoint"};
        for (size_t i = 0; i < sizeof(water) / sizeof(water[0]); i++) {
            status = li8x0_set("rs232", water[i], NULL, "TRUE");
            if (status != LI8X0_OK) return status;
        }
    }
    return LI8X0_OK;
}

li8x0_status_t li8x0_autodiscover(void)
{
    if (pending) return LI8X0_BUSY;
    strcpy(tx_buf, "<who>\r\n");
    li8x0_status_t status = begin(true, LI8X0_TIMEOUT_MS);
    if (status != LI8X0_OK) return status;
    size_t scan = 0;
    size_t previous_total = 0;
    uint32_t changed = HAL_GetTick();
    li8x0_model_t discovered = LI8X0_UNKNOWN;
    while ((uint32_t)(HAL_GetTick() - request_started) < LI8X0_TIMEOUT_MS) {
        if (li8x0_huart->ErrorCode != HAL_UART_ERROR_NONE) return finish(LI8X0_UART_ERROR);
        size_t remaining = __HAL_DMA_GET_COUNTER(li8x0_huart->hdmarx);
        if (remaining > sizeof(rx_buf) - 1) return finish(LI8X0_UART_ERROR);
        size_t total = sizeof(rx_buf) - 1 - remaining;
        __DMB();
        uint32_t now = HAL_GetTick();
        if (total != previous_total) { changed = now; previous_total = total; }
        while (scan + 7 <= total) {
            if (rx_buf[scan] == '<' && rx_buf[scan + 6] == '>') {
                char name[6];
                for (size_t i = 0; i < 5; i++) name[i] = (char)tolower(rx_buf[scan + 1 + i]);
                name[5] = '\0';
                li8x0_model_t model = name_model(name);
                if (model != LI8X0_UNKNOWN) {
                    if (discovered != LI8X0_UNKNOWN && model != discovered)
                        return finish(LI8X0_REJECTED);
                    discovered = model;
                }
            }
            scan++;
        }
        if (discovered != LI8X0_UNKNOWN && (uint32_t)(now - changed) >= LI8X0_REPLY_GAP_MS) {
            status = finish(LI8X0_OK);
            if (status == LI8X0_OK) li8x0_model = discovered;
            return status;
        }
        if (total == sizeof(rx_buf) - 1) return finish(LI8X0_OVERFLOW);
    }
    return finish(LI8X0_TIMEOUT);
}

static bool valid_date(const char *date)
{
    if (!date || strlen(date) != 10 || date[4] != '-' || date[7] != '-') return false;
    unsigned year = 0, month = 0, day = 0;
    for (size_t i = 0; i < 10; i++) {
        if (i == 4 || i == 7) continue;
        if (!isdigit((unsigned char)date[i])) return false;
        unsigned digit = (unsigned)(date[i] - '0');
        if (i < 4) year = year * 10 + digit;
        else if (i < 7) month = month * 10 + digit;
        else day = day * 10 + digit;
    }
    static const unsigned days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (!year || !month || month > 12 || !day) return false;
    unsigned limit = days[month - 1];
    if (month == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) limit++;
    return day <= limit;
}

static li8x0_status_t start_calibration(const char *date, const char *field, const char *value)
{
    if (pending) return LI8X0_BUSY;
    if (!valid_date(date)) return LI8X0_INVALID_ARG;
    const char *root = model_name(li8x0_model);
    if (!root) return LI8X0_NOT_READY;
    int n = snprintf(tx_buf, sizeof(tx_buf),
                     "<%s><cal><date>%s</date><%s>%s</%s></cal></%s>\r\n",
                     root, date, field, value, field, root);
    li8x0_status_t status = command_size(n);
    if (status == LI8X0_OK) status = begin(false, LI8X0_TIMEOUT_MS);
    return status == LI8X0_OK ? receive("", true, NULL) : status;
}

li8x0_status_t li8x0_start_zero(const char *date)
{
    return start_calibration(date, "co2zero", "TRUE");
}

li8x0_status_t li8x0_start_span(const char *date, float ppm, bool secondary)
{
    if (!isfinite(ppm) || ppm <= 0) return LI8X0_INVALID_ARG;
    char text[48];
    int n = snprintf(text, sizeof(text), "%.8g", (double)ppm);
    if (n < 0 || (size_t)n >= sizeof(text)) return LI8X0_OVERFLOW;
    return start_calibration(date, secondary ? "co2span2" : "co2span", text);
}
