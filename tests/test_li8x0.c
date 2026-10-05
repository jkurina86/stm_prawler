#include "LI8x0.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Deliberate protocol fixtures, not captured hardware measurements. */
static const char sample[] =
    "<li830><data><co2>412.25</co2><celltemp>51.2</celltemp>"
    "<cellpres>101.32</cellpres><ivolt>12.1</ivolt><raw>"
    "<co2>123456</co2><co2ref>-42</co2ref></raw></data></li830>";
static const char ack[] = "<li830><ack>TRUE</ack></li830>\r\n";
static const char nack[] = "<li830><ack>FALSE</ack></li830>\r\n";
static DMA_HandleTypeDef dma;
static UART_HandleTypeDef uart;
static uint32_t tick;
static uint8_t *dma_buffer;
static uint16_t dma_size;
static size_t delivered;
static const char *reply;
static char combined[LI8X0_RX_BUF_SIZE + 128];
static char sent[LI8X0_TX_BUF_SIZE];
static char sent_history[32][LI8X0_TX_BUF_SIZE];
static char automatic_reply[64];
static HAL_StatusTypeDef rx_status, tx_status;
static bool armed;
static bool waiting_for_tx;
static bool automatic_ack;
static bool inject_error;
static unsigned transmitted;
static unsigned chunk_size;
static bool wire_mode;
static const char *wire_reply_on_tx;
static struct {
    const char *text;
    size_t position;
    uint32_t ready;
} wire[8];
static unsigned wire_count, wire_head;

static void wire_add(const char *text, uint32_t delay)
{
    assert(wire_count < sizeof(wire) / sizeof(wire[0]));
    wire[wire_count].text = text;
    wire[wire_count].position = 0;
    wire[wire_count++].ready = tick + delay;
}

static void advance(void)
{
    tick++;
    if (wire_mode && wire_head < wire_count &&
        (int32_t)(tick - wire[wire_head].ready) >= 0) {
        /* Sensor transmission continues across aborts and DMA re-arms. */
        for (unsigned i = 0; i < chunk_size && wire[wire_head].text[wire[wire_head].position]; i++) {
            uint8_t byte = (uint8_t)wire[wire_head].text[wire[wire_head].position++];
            if (armed && dma.counter) {
                dma_buffer[dma_size - dma.counter] = byte;
                dma.counter--;
            }
        }
        if (!wire[wire_head].text[wire[wire_head].position]) wire_head++;
    }
    if (armed && !waiting_for_tx && inject_error) {
        uart.ErrorCode = 1;
        armed = false;
    }
    if (!wire_mode && armed && !waiting_for_tx && reply && reply[delivered] && delivered < dma_size) {
        /* Deliver fragmented replies, including gaps shorter than reply-gap limit. */
        if (tick % 3 != 0) {
            for (unsigned i = 0; i < chunk_size && reply[delivered] && delivered < dma_size; i++) {
                dma_buffer[delivered] = (uint8_t)reply[delivered];
                delivered++;
            }
            dma.counter = dma_size - (uint32_t)delivered;
        }
    }
}

uint32_t HAL_GetTick(void) { advance(); return tick; }
void HAL_Delay(uint32_t ms) { while (ms--) advance(); }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u, uint8_t *buf, uint16_t len)
{
    assert(u == &uart);
    if (rx_status != HAL_OK) return rx_status;
    dma_buffer = buf;
    dma_size = len;
    dma.counter = len;
    u->ErrorCode = HAL_UART_ERROR_NONE;
    delivered = 0;
    armed = true;
    waiting_for_tx = !wire_mode;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *buf,
                                  uint16_t len, uint32_t timeout)
{
    assert(u == &uart && armed && timeout > 0);
    assert(len < sizeof(sent));
    memcpy(sent, buf, len);
    sent[len] = '\0';
    assert(transmitted < sizeof(sent_history) / sizeof(sent_history[0]));
    strcpy(sent_history[transmitted], sent);
    transmitted++;
    waiting_for_tx = false;
    if (wire_mode && wire_reply_on_tx) wire_add(wire_reply_on_tx, 20);
    if (automatic_ack) {
        unsigned model = (unsigned)li8x0_get_model();
        snprintf(automatic_reply, sizeof(automatic_reply),
                 "<li%u><ack>TRUE</ack></li%u>\r\n", model, model);
        reply = automatic_reply;
    }
    return tx_status;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{
    assert(u == &uart);
    armed = false;
    return HAL_OK;
}

static void reset(void)
{
    li8x0_cancel();
    memset(&uart, 0, sizeof(uart));
    memset(&dma, 0, sizeof(dma));
    uart.hdmarx = &dma;
    dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma.Init.PeriphInc = DMA_PINC_DISABLE;
    dma.Init.MemInc = DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uart.Init.BaudRate = 9600;
    uart.Init.Mode = UART_MODE_TX_RX;
    tick = 0;
    reply = NULL;
    sent[0] = '\0';
    delivered = 0;
    rx_status = tx_status = HAL_OK;
    armed = automatic_ack = inject_error = false;
    transmitted = 0;
    chunk_size = 8;
    wire_mode = false;
    wire_reply_on_tx = NULL;
    wire_count = wire_head = 0;
    assert(li8x0_init(&uart, LI8X0_LI830) == LI8X0_OK);
    assert(li8x0_resync(0) == LI8X0_OK);
}

static void test_parser(void)
{
    li8x0_response_t out;
    assert(li8x0_parse_response(sample, strlen(sample), &out) == LI8X0_OK);
    assert(out.model == LI8X0_LI830 && out.data_present);
    assert(out.data.co2 == 412.25f && out.data.raw.co2 == 123456);
    assert(out.data.raw.co2ref == -42 && out.data.ivolt == 12.1f);
    assert(isnan(out.data.h2o));
    assert((out.data.valid_fields & LI8X0_DATA_RAW_CO2) != 0);

    const char water[] = " <LI850><DATA><CO2>-1.25e1</CO2><H2O>2.5</H2O>"
                        "<H2ODEWPOINT>18</H2ODEWPOINT><RAW><H2O>9</H2O>"
                        "<H2OREF>10</H2OREF></RAW></DATA></LI850>\r\n";
    assert(li8x0_parse_response(water, sizeof(water) - 1, &out) == LI8X0_OK);
    assert(out.data.co2 == -12.5f && out.data.h2o == 2.5f);
    assert(out.data.h2odewpoint == 18 && out.data.raw.h2oref == 10);

    const char identity[] = "<li830><serialnum>HGa-a123</serialnum>"
                            "<ver>v1.2Beta</ver><error></error></li830>";
    assert(li8x0_parse_response(identity, sizeof(identity) - 1, &out) == LI8X0_OK);
    assert(strcmp(out.serialnum, "HGa-a123") == 0);
    assert(strcmp(out.version, "v1.2Beta") == 0);
    assert(li8x0_parse_response(nack, strlen(nack), &out) == LI8X0_OK);
    assert(out.ack_present && !out.ack);

    const char calibration[] = "<li830><cal><co2kzero>1.001</co2kzero>"
        "<co2kspan>0.99</co2kspan><co2kspan2>1.02</co2kspan2>"
        "<co2lastzero>2026-10-04</co2lastzero></cal></li830>";
    assert(li8x0_parse_response(calibration, sizeof(calibration) - 1, &out) == LI8X0_OK);
    assert(out.cal_present && out.cal.valid_fields == 7);
    assert(out.cal.co2kspan == 0.99f && isnan(out.cal.h2okspan));
    assert(strcmp(out.cal.co2lastzero, "2026-10-04") == 0);

    /* Every truncated prefix must fail without publishing a partial measurement. */
    li8x0_response_t saved = out;
    for (size_t i = 0; i < strlen(sample); i++) {
        assert(li8x0_parse_response(sample, i, &out) != LI8X0_OK);
        assert(memcmp(&out, &saved, sizeof(out)) == 0);
    }
    const char *bad[] = {
        "<li830><data><co2>12junk</co2></data></li830>",
        "<li830><data><co2>nan</co2></data></li830>",
        "<li830><data><co2>inf</co2></data></li830>",
        "<li830><data><co2>1e99</co2></data></li830>",
        "<li830><data><co2>0x1p2</co2></data></li830>",
        "<li830><data><co2>1</co2><co2>2</co2></data></li830>",
        "<li830><data><co2>1</co2></wrong></li830>",
        "<li830><data><raw><co2>2147483648</co2></raw></data></li830>",
        "<li830><ack>TRUEjunk</ack></li830>",
        "<li830><ack>TRUE</ack><ack>FALSE</ack></li830>",
        "<li830><serialnum>12345678901234567890123456789012</serialnum></li830>",
        "<li830><data>mixed<co2>1</co2></data></li830>",
        "<li830><data><co2><x>1</x></co2></data></li830>",
        "<li830><a><b><c><d><e><f>1</f></e></d></c></b></a></li830>",
        "<li999><data><co2>1</co2></data></li999>",
        "<li830></li830>garbage"
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
        assert(li8x0_parse_response(bad[i], strlen(bad[i]), &out) != LI8X0_OK);
    assert(li8x0_parse_response(NULL, 1, &out) == LI8X0_INVALID_ARG);
    assert(li8x0_parse_response(sample, strlen(sample), NULL) == LI8X0_INVALID_ARG);
}

static void test_transport(void)
{
    reset();
    reply = sample;
    chunk_size = 1;
    li8x0_data_t data;
    assert(li8x0_sample(&data) == LI8X0_OK);
    assert(data.co2 == 412.25f && !armed);
    assert(strcmp(sent, "<li830><data>?</data></li830>\r\n") == 0);
    size_t len;
    assert(strcmp(li8x0_last_response(&len), sample) == 0 && len == strlen(sample));

    reset();
    snprintf(combined, sizeof(combined), "%s%s%s", ack, sample, ack);
    reply = combined;
    assert(li8x0_fire() == LI8X0_OK);
    assert(li8x0_fire() == LI8X0_BUSY);
    assert(li8x0_init(&uart, LI8X0_LI850) == LI8X0_BUSY);
    assert(li8x0_set("cfg", "heater", NULL, "TRUE") == LI8X0_BUSY);
    assert(li8x0_collect(&data) == LI8X0_OK && data.co2 == 412.25f);
    assert(li8x0_collect(&data) == LI8X0_NOT_READY);

    reset();
    reply = ack;
    li8x0_clear_data(&data);
    data.co2 = 99;
    assert(li8x0_sample(&data) == LI8X0_TIMEOUT);
    assert(data.co2 == 99 && !armed);
    assert(li8x0_fire() == LI8X0_SYNC_REQUIRED);
    reply = NULL;
    assert(li8x0_resync(0) == LI8X0_OK);
    reply = sample;
    assert(li8x0_sample(&data) == LI8X0_OK);

    reset();
    reply = sample;
    assert(li8x0_fire() == LI8X0_OK);
    HAL_Delay(LI8X0_TIMEOUT_MS + 1);
    assert(li8x0_collect(&data) == LI8X0_TIMEOUT);
    reset();
    assert(li8x0_fire() == LI8X0_OK);
    li8x0_cancel();
    assert(!armed && li8x0_collect(&data) == LI8X0_NOT_READY);

    reset();
    reply = nack;
    assert(li8x0_heater(true) == LI8X0_REJECTED);
    reply = NULL;
    assert(li8x0_resync(0) == LI8X0_OK);
    reply = "<li830><error>Bad setting</error></li830>";
    assert(li8x0_heater(false) == LI8X0_REJECTED);
    reset();
    reply = "<li850><data><co2>1</co2></data></li850>";
    assert(li8x0_sample(&data) == LI8X0_REJECTED);
    reset();
    reply = "<li830><data><raw><co2>1</co2></raw></data></li830>";
    assert(li8x0_sample(&data) == LI8X0_PARSE_ERROR);
    reset();
    rx_status = HAL_BUSY;
    assert(li8x0_fire() == LI8X0_BUSY && transmitted == 0);
    rx_status = HAL_ERROR;
    assert(li8x0_fire() == LI8X0_UART_ERROR);
    reset();
    tx_status = HAL_ERROR;
    assert(li8x0_fire() == LI8X0_UART_ERROR && !armed);
    reset();
    inject_error = true;
    assert(li8x0_sample(&data) == LI8X0_UART_ERROR && !armed);
    reset();
    memset(combined, 'x', LI8X0_RX_BUF_SIZE + 5);
    combined[LI8X0_RX_BUF_SIZE + 5] = '\0';
    reply = combined;
    assert(li8x0_sample(&data) == LI8X0_OVERFLOW && !armed);

    reset();
    tick = UINT32_MAX - 100;
    reply = sample;
    assert(li8x0_sample(&data) == LI8X0_OK);
    assert(li8x0_sample(NULL) == LI8X0_INVALID_ARG);
    assert(li8x0_init(NULL, LI8X0_LI830) == LI8X0_INVALID_ARG);
    dma.Init.Mode = DMA_CIRCULAR;
    assert(li8x0_init(&uart, LI8X0_LI830) == LI8X0_INVALID_ARG);
    dma.Init.Mode = DMA_NORMAL;
    uart.Init.BaudRate = 19200;
    assert(li8x0_init(&uart, LI8X0_LI830) == LI8X0_INVALID_ARG);
}

static void test_commands(void)
{
    reset();
    automatic_ack = true;
    assert(li8x0_heater(true) == LI8X0_OK);
    assert(strcmp(sent, "<li830><cfg><heater>TRUE</heater></cfg></li830>\r\n") == 0);
    assert(li8x0_set("rs232", "raw", "co2", "TRUE") == LI8X0_OK);
    assert(strstr(sent, "<raw><co2>TRUE</co2></raw>") != NULL);
    unsigned before = transmitted;
    assert(li8x0_set("cfg>", "heater", NULL, "TRUE") == LI8X0_INVALID_ARG);
    assert(li8x0_set("cfg", "heater", NULL, "<x>") == LI8X0_INVALID_ARG);
    assert(li8x0_set_co2_span(NAN) == LI8X0_INVALID_ARG);
    assert(li8x0_start_zero("2026-02-30") == LI8X0_INVALID_ARG);
    assert(transmitted == before);
    assert(li8x0_start_zero("2026-10-04") == LI8X0_OK);
    assert(strstr(sent, "<date>2026-10-04</date><co2zero>TRUE</co2zero>") != NULL);
    assert(li8x0_start_span("2024-02-29", 400.5f, true) == LI8X0_OK);
    assert(strstr(sent, "<co2span2>400.5</co2span2>") != NULL);
    assert(li8x0_start_span("2025-02-29", 400, false) == LI8X0_INVALID_ARG);
    assert(li8x0_configure() == LI8X0_OK);

    reset();
    reply = nack;
    assert(li8x0_configure() == LI8X0_REJECTED && transmitted == 1);
    reset();
    reply = "<li830><serialnum>HGA-42</serialnum><ver>v2.1</ver></li830>\r\n";
    li8x0_response_t out;
    assert(li8x0_get_info(&out) == LI8X0_OK && strcmp(out.serialnum, "HGA-42") == 0);
    reply = "<li830><cal><co2kzero>1.25</co2kzero></cal></li830>";
    li8x0_cal_t cal;
    assert(li8x0_get_cal(&cal) == LI8X0_OK && cal.co2kzero == 1.25f);
    reset();
    reply = "<li830><cfg><heater>TRUE</heater></cfg></li830>";
    assert(li8x0_query("poly", &out) == LI8X0_TIMEOUT);
    reset();
    reply = "<li830>\r\n";
    assert(li8x0_init(&uart, LI8X0_UNKNOWN) == LI8X0_OK);
    assert(li8x0_fire() == LI8X0_NOT_READY);
    assert(li8x0_autodiscover() == LI8X0_OK && li8x0_get_model() == LI8X0_LI830);
    assert(strcmp(sent, "<who>\r\n") == 0);
    reset();
    reply = sample;
    assert(li8x0_autodiscover() == LI8X0_OK && delivered == strlen(sample));

    reset();
    reply = sample;
    assert(li8x0_query("DATA", &out) == LI8X0_OK && out.data.co2 == 412.25f);

    const li8x0_model_t models[] = {LI8X0_LI820, LI8X0_LI830, LI8X0_LI840, LI8X0_LI850};
    for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); i++) {
        reset();
        assert(li8x0_init(&uart, models[i]) == LI8X0_OK);
        automatic_ack = true;
        assert(li8x0_configure() == LI8X0_OK);
        bool water_enabled = false, flow_disabled = false;
        for (unsigned j = 0; j < transmitted; j++) {
            assert(strstr(sent_history[j], "<heater>") == NULL);
            assert(strstr(sent_history[j], "<pcomp>") == NULL);
            assert(strstr(sent_history[j], "<cal>") == NULL);
            water_enabled |= strstr(sent_history[j], "<h2o>TRUE</h2o>") != NULL;
            flow_disabled |= strstr(sent_history[j], "<flowrate>FALSE</flowrate>") != NULL;
        }
        assert(water_enabled == (models[i] == LI8X0_LI840 || models[i] == LI8X0_LI850));
        assert(flow_disabled == (models[i] == LI8X0_LI830 || models[i] == LI8X0_LI850));
    }
}

static void test_dma_configuration(void)
{
    reset();
    uint32_t *fields[] = {&dma.Init.Direction, &dma.Init.PeriphInc, &dma.Init.MemInc,
                         &dma.Init.PeriphDataAlignment, &dma.Init.MemDataAlignment};
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        uint32_t saved = *fields[i];
        *fields[i] ^= 1;
        assert(li8x0_init(&uart, LI8X0_LI830) == LI8X0_INVALID_ARG);
        assert(li8x0_fire() == LI8X0_INVALID_ARG && transmitted == 0);
        *fields[i] = saved;
    }
}

static void test_unsolicited_full_state(void)
{
    reset();
    wire_mode = true;
    wire_add(sample, 5);
    wire_add("<li830><serialnum>HGA-42</serialnum><ver>v2.1</ver></li830>", 300);
    li8x0_response_t out;
    assert(li8x0_get_info(&out) == LI8X0_OK);
    assert(strcmp(out.serialnum, "HGA-42") == 0 && wire_head == wire_count);
}

static void test_stale_ack(void)
{
    reset();
    wire_mode = true;
    assert(li8x0_fire() == LI8X0_OK);
    wire_add(ack, 50);
    li8x0_cancel();
    wire_reply_on_tx = nack;
    assert(li8x0_heater(false) == LI8X0_SYNC_REQUIRED && transmitted == 1);
    assert(li8x0_init(&uart, LI8X0_LI830) == LI8X0_OK);
    assert(li8x0_heater(false) == LI8X0_SYNC_REQUIRED && transmitted == 1);
    assert(li8x0_resync(200) == LI8X0_OK && wire_head == wire_count);
    /* Only the new setting's NACK remains after explicit resynchronization. */
    assert(li8x0_heater(false) == LI8X0_REJECTED);

    reset();
    wire_mode = true;
    wire_add("<li830><data><co2>bad</co2></data></li830>", 10);
    wire_add(ack, 80);
    li8x0_data_t data;
    assert(li8x0_sample(&data) == LI8X0_PARSE_ERROR);
    assert(li8x0_heater(false) == LI8X0_SYNC_REQUIRED && transmitted == 1);
    assert(li8x0_resync(200) == LI8X0_OK && wire_head == wire_count);
    wire_reply_on_tx = nack;
    assert(li8x0_heater(false) == LI8X0_REJECTED);

    reset();
    assert(li8x0_resync(LI8X0_QUERY_TIMEOUT_MS + 1) == LI8X0_INVALID_ARG);
    assert(li8x0_fire() == LI8X0_OK);
    assert(li8x0_resync(0) == LI8X0_BUSY);
    li8x0_cancel();
    rx_status = HAL_ERROR;
    assert(li8x0_resync(0) == LI8X0_UART_ERROR);
    assert(li8x0_fire() == LI8X0_SYNC_REQUIRED && transmitted == 1);
    rx_status = HAL_OK;
    assert(li8x0_resync(0) == LI8X0_OK);

    reset();
    wire_mode = true;
    chunk_size = 1;
    memset(combined, 'x', LI8X0_RX_BUF_SIZE + 5);
    combined[LI8X0_RX_BUF_SIZE + 5] = '\0';
    wire_add(combined, 1);
    assert(li8x0_resync(0) == LI8X0_OVERFLOW);
    assert(li8x0_fire() == LI8X0_SYNC_REQUIRED && transmitted == 0);
}

static void test_parser_mutations(void)
{
    char input[sizeof(sample)];
    const unsigned char mutations[] = {0, '<', '>', '/', ' ', 0xff};
    li8x0_response_t out;
    memset(&out, 0x5a, sizeof(out));
    for (size_t i = 0; i < sizeof(sample) - 1; i++) {
        for (size_t j = 0; j < sizeof(mutations); j++) {
            memcpy(input, sample, sizeof(input));
            input[i] = (char)mutations[j];
            li8x0_response_t saved = out;
            li8x0_status_t status = li8x0_parse_response(input, sizeof(input) - 1, &out);
            if (status != LI8X0_OK) assert(memcmp(&out, &saved, sizeof(out)) == 0);
        }
    }
}

int main(void)
{
    test_parser();
    test_transport();
    test_commands();
    test_dma_configuration();
    test_unsolicited_full_state();
    test_stale_ack();
    test_parser_mutations();
    puts("LI8x0: parser, fragmented DMA transport, timeouts/errors, commands PASS");
    return 0;
}
