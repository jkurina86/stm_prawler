/**
 * @file licor_shell.c
 * @brief LI-COR commands on the exclusively owned USART2 interface.
 * Called from shell_task/foreground dispatch, never from an ISR. Arguments are
 * consumed before the shell buffer is reset; no argv pointers are queued.
 */
#include "licor_shell.h"
#include "LI8x0.h"
#include "config.h"
#include "main.h"
#include "shell.h"
#include <errno.h>

extern UART_HandleTypeDef huart2;

static const char *status_name(li8x0_status_t status)
{
    switch (status) {
    case LI8X0_OK: return "OK";
    case LI8X0_INVALID_ARG: return "invalid argument";
    case LI8X0_NOT_READY: return "UART unbound or model unknown";
    case LI8X0_BUSY: return "busy";
    case LI8X0_TIMEOUT: return "timeout";
    case LI8X0_UART_ERROR: return "UART error";
    case LI8X0_OVERFLOW: return "buffer overflow";
    case LI8X0_PARSE_ERROR: return "invalid XML response";
    case LI8X0_REJECTED: return "instrument rejected command";
    case LI8X0_SYNC_REQUIRED: return "resynchronization required";
    default: return "unknown error";
    }
}

static bool report_status(const char *command, li8x0_status_t status)
{
    if (status == LI8X0_OK) {
        g_app.licor_status = PERIPH_READY;
        return true;
    }
    shell_printf("[%s] %s (err %u)\r\n", command, status_name(status), (unsigned)status);
    if (status != LI8X0_INVALID_ARG && status != LI8X0_BUSY)
        g_app.licor_status = PERIPH_ERROR;
    if (status == LI8X0_REJECTED) {
        const char *xml = li8x0_last_response(NULL);
        if (xml) { shell_print(xml); shell_print("\r\n"); }
    }
    if (status == LI8X0_TIMEOUT || status == LI8X0_UART_ERROR ||
        status == LI8X0_OVERFLOW || status == LI8X0_PARSE_ERROR ||
        status == LI8X0_REJECTED || status == LI8X0_SYNC_REQUIRED) {
        shell_print("After the previous operation ends or the instrument is reset, "
                    "use licor-resync before another request.\r\n");
    }
    return false;
}

static bool ensure_model(const char *command)
{
    return li8x0_get_model() != LI8X0_UNKNOWN ||
           report_status(command, li8x0_autodiscover());
}

static bool no_arguments(int argc, const char *command)
{
    if (argc == 1) return true;
    shell_printf("Usage: %s\r\n", command);
    return false;
}

static void print_xml(void)
{
    /* shell_printf has a 256-byte formatting buffer; XML may be 4095 bytes. */
    const char *xml = li8x0_last_response(NULL);
    if (xml) {
        shell_print(xml);
        shell_print("\r\n");
    }
}

void cmd_licor(int argc, char **argv)
{
    (void)argv;
    if (!no_arguments(argc, "licor") || !ensure_model("licor")) return;
    li8x0_data_t data;
    if (!report_status("licor", li8x0_sample(&data))) return;
    shell_printf("LI-%u measurements:\r\n", (unsigned)li8x0_get_model());
#define PRINT_FIELD(bit, label, field, unit) \
    if (data.valid_fields & (bit)) shell_printf("  " label ": %.4f " unit "\r\n", (double)data.field)
    PRINT_FIELD(LI8X0_DATA_CO2, "CO2", co2, "ppm");
    PRINT_FIELD(LI8X0_DATA_CO2ABS, "CO2 absorbance", co2abs, "");
    PRINT_FIELD(LI8X0_DATA_H2O, "H2O", h2o, "mmol/mol");
    PRINT_FIELD(LI8X0_DATA_H2ODEW, "H2O dew point", h2odewpoint, "C");
    PRINT_FIELD(LI8X0_DATA_H2OABS, "H2O absorbance", h2oabs, "");
    PRINT_FIELD(LI8X0_DATA_CELLTEMP, "Cell temperature", celltemp, "C");
    PRINT_FIELD(LI8X0_DATA_CELLPRES, "Cell pressure", cellpres, "kPa");
    PRINT_FIELD(LI8X0_DATA_IVOLT, "Input voltage", ivolt, "V");
    PRINT_FIELD(LI8X0_DATA_FLOWRATE, "Flow rate", flowrate, "");
#undef PRINT_FIELD
#define PRINT_RAW(bit, label, field) \
    if (data.valid_fields & (bit)) shell_printf("  " label ": %ld\r\n", (long)data.raw.field)
    PRINT_RAW(LI8X0_DATA_RAW_CO2, "Raw CO2", co2);
    PRINT_RAW(LI8X0_DATA_RAW_CO2REF, "Raw CO2 reference", co2ref);
    PRINT_RAW(LI8X0_DATA_RAW_H2O, "Raw H2O", h2o);
    PRINT_RAW(LI8X0_DATA_RAW_H2OREF, "Raw H2O reference", h2oref);
#undef PRINT_RAW
}

void cmd_licor_raw(int argc, char **argv)
{
    (void)argv;
    if (!no_arguments(argc, "licor-raw") || !ensure_model("licor-raw")) return;
    li8x0_data_t data;
    if (report_status("licor-raw", li8x0_sample(&data))) print_xml();
}

void cmd_licor_info(int argc, char **argv)
{
    (void)argv;
    if (!no_arguments(argc, "licor-info") || !ensure_model("licor-info")) return;
    li8x0_response_t response;
    if (!report_status("licor-info", li8x0_get_info(&response))) return;
    shell_printf("LI-%u\r\nSerial: %s\r\nFirmware: %s\r\n",
                 (unsigned)response.model, response.serialnum, response.version);
}

void cmd_licor_discover(int argc, char **argv)
{
    (void)argv;
    if (!no_arguments(argc, "licor-discover")) return;
    if (report_status("licor-discover", li8x0_autodiscover()))
        shell_printf("Discovered LI-%u on USART2\r\n", (unsigned)li8x0_get_model());
}

void cmd_licor_model(int argc, char **argv)
{
    if (argc == 1) {
        li8x0_model_t model = li8x0_get_model();
        if (model == LI8X0_UNKNOWN) shell_print("LI-COR model: undiscovered\r\n");
        else shell_printf("LI-COR model: LI-%u\r\n", (unsigned)model);
        return;
    }
    li8x0_model_t model = LI8X0_UNKNOWN;
    if (argc == 2) {
        if (!strcmp(argv[1], "820")) model = LI8X0_LI820;
        else if (!strcmp(argv[1], "830")) model = LI8X0_LI830;
        else if (!strcmp(argv[1], "840")) model = LI8X0_LI840;
        else if (!strcmp(argv[1], "850")) model = LI8X0_LI850;
    }
    if (model == LI8X0_UNKNOWN) {
        shell_print("Usage: licor-model [820|830|840|850]\r\n");
        return;
    }
    li8x0_status_t status = li8x0_init(&huart2, model);
    if (status == LI8X0_OK)
        shell_printf("LI-COR model selected: LI-%u (no discovery sent)\r\n", (unsigned)model);
    else
        (void)report_status("licor-model", status);
}

void cmd_licor_configure(int argc, char **argv)
{
    (void)argv;
    if (!no_arguments(argc, "licor-configure") || !ensure_model("licor-configure")) return;
    if (report_status("licor-configure", li8x0_configure()))
        shell_print("LI-COR polled XML output configured\r\n");
}

void cmd_licor_query(int argc, char **argv)
{
    if (argc > 2) {
        shell_print("Usage: licor-query [section]\r\n");
        return;
    }
    const char *section = argc == 2 ? argv[1] : "";
    /* Validate before discovery so invalid shell input never sends a request. */
    if (strlen(section) > 23 || strspn(section, "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz0123456789_") != strlen(section)) {
        shell_print("Invalid LI-COR section name\r\n");
        return;
    }
    if (!ensure_model("licor-query")) return;
    li8x0_response_t response;
    if (report_status("licor-query", li8x0_query(section, &response))) print_xml();
}

void cmd_licor_set(int argc, char **argv)
{
    if (argc != 4 && argc != 5) {
        shell_print("Usage: licor-set section field [subfield] value\r\n");
        return;
    }
    /* Match the driver's bounded scalar grammar before any automatic discovery. */
    for (int i = 1; i < argc; i++) {
        const char *allowed = i == argc - 1 ?
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789.+-?_:" :
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_";
        size_t limit = i == argc - 1 ? 64 : 23;
        if (!argv[i][0] || strlen(argv[i]) > limit || strspn(argv[i], allowed) != strlen(argv[i])) {
            shell_print("Invalid LI-COR field/value\r\n");
            return;
        }
    }
    if (!ensure_model("licor-set")) return;
    const char *subfield = argc == 5 ? argv[3] : NULL;
    if (report_status("licor-set", li8x0_set(argv[1], argv[2], subfield, argv[argc - 1])))
        shell_print("LI-COR setting acknowledged\r\n");
}

void cmd_licor_heater(int argc, char **argv)
{
    if (argc != 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
        shell_print("Usage: licor-heater on|off\r\n");
        return;
    }
    if (!ensure_model("licor-heater")) return;
    if (report_status("licor-heater", li8x0_heater(!strcmp(argv[1], "on"))))
        shell_print("LI-COR heater setting acknowledged\r\n");
}

void cmd_licor_resync(int argc, char **argv)
{
    unsigned long settle_ms = 0;
    if (argc == 2) {
        char *end;
        errno = 0;
        settle_ms = strtoul(argv[1], &end, 10);
        if (errno || !argv[1][0] || *end ||
            strspn(argv[1], "0123456789") != strlen(argv[1]) || settle_ms > 5000) {
            shell_print("Usage: licor-resync [0-5000 ms]\r\n");
            return;
        }
    } else if (argc != 1) {
        shell_print("Usage: licor-resync [0-5000 ms]\r\n");
        return;
    }
    if (report_status("licor-resync", li8x0_resync((uint32_t)settle_ms)))
        shell_print("LI-COR RX synchronized; requests enabled\r\n");
}
