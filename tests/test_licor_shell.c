/* Exercise the real shell/parser/LI-COR driver with a simulated USART2. */
#include "main.h"
#include "LI8x0.h"
#include "shell.h"
#include "task_handlers.h"
#include "tasker.h"
#include "config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1, huart2;
static DMA_HandleTypeDef dma;
static char output[16384], sent[LI8X0_TX_BUF_SIZE];
static const char sample[] = "<li830><data><co2>412.25</co2><celltemp>51.2</celltemp>"
                             "<raw><co2>123</co2></raw></data></li830>";
static char long_reply[1024];
static const char *reply;
static uint8_t *rx;
static uint16_t capacity;
static size_t delivered;
static uint32_t tick;
static unsigned requests;
static bool armed, silence;

uint32_t HAL_GetTick(void)
{
    tick++;
    if (armed && reply && reply[delivered]) {
        for (unsigned i = 0; i < 16 && reply[delivered] && dma.counter; i++) {
            rx[delivered] = (uint8_t)reply[delivered];
            delivered++;
            dma.counter--;
        }
    }
    return tick;
}
void HAL_Delay(uint32_t ms) { while (ms--) (void)HAL_GetTick(); }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size)
{
    assert(uart == &huart2);
    rx = buffer;
    capacity = size;
    dma.counter = size;
    delivered = 0;
    reply = NULL;
    armed = true;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(uart == &huart2);
    armed = false;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size)
{
    assert(uart == &huart1 && buffer && size == 1);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *buffer,
                                  uint16_t size, uint32_t timeout)
{
    assert(timeout > 0);
    if (uart == &huart1) {
        size_t offset = strlen(output);
        assert(offset + size < sizeof(output));
        memcpy(output + offset, buffer, size);
        output[offset + size] = '\0';
        return HAL_OK;
    }
    assert(uart == &huart2 && armed && capacity > 0 && size < sizeof(sent));
    memcpy(sent, buffer, size);
    sent[size] = '\0';
    requests++;
    if (silence) return HAL_OK;
    if (!strcmp(sent, "<who>\r\n")) reply = "<li830>";
    else if (strstr(sent, "<data>?</data>")) reply = sample;
    else if (!strcmp(sent, "<li830>?</li830>\r\n"))
        reply = "<li830><serialnum>83TEST</serialnum><ver>1.23</ver></li830>";
    else if (strstr(sent, "<cfg>?</cfg>")) reply = long_reply;
    else if (strstr(sent, "<cal>?</cal>"))
        reply = "<li830><cal><co2kzero>1.25</co2kzero></cal></li830>";
    else reply = "<li830><ack>TRUE</ack></li830>";
    return HAL_OK;
}
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port; (void)pin;
    return GPIO_PIN_RESET;
}

/* Unrelated handlers are stand-ins; their command registration is tested below. */
#define HANDLER(name) void handle_##name(const void *arg) { (void)arg; }
HANDLER(help) HANDLER(clear) HANDLER(status) HANDLER(version) HANDLER(settime)
HANDLER(rtc_settime) HANDLER(rtc_gettime) HANDLER(rtc_epoch) HANDLER(rtc_temp)
HANDLER(rtc_timer_set) HANDLER(rtc_timer_stop) HANDLER(rtc_timer_status)
HANDLER(fs_mount) HANDLER(fs_unmount) HANDLER(fs_ls) HANDLER(who)
#undef HANDLER

static bool registered(const char *name)
{
    for (unsigned i = 0; shell_commands[i].name; i++)
        if (!strcmp(shell_commands[i].name, name)) return true;
    return false;
}
static void command(const char *text)
{
    char line[256];
    assert(strlen(text) < sizeof(line));
    strcpy(line, text);
    output[0] = '\0';
    assert(shell_dispatch(line));
}
static void invalid(const char *text)
{
    unsigned before = requests;
    command(text);
    assert(requests == before && output[0]);
}

int main(void)
{
    config_init();
    tasker_init();
    dma.Init.Mode = DMA_NORMAL;
    dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma.Init.MemInc = DMA_MINC_ENABLE;
    huart2.hdmarx = &dma;
    huart2.Init.BaudRate = 9600;
    huart2.Init.Mode = UART_MODE_TX_RX;
    assert(li8x0_init(&huart2, LI8X0_UNKNOWN) == LI8X0_OK);
    const char *removed[] = { "ctd", "ctd-raw", "optode", "optode-raw", "optode-listen",
        "wetlab", "wetlab-raw", "sensors", "config", "samplerate", "wifi-up", "wifi-down",
        "wifi-status", "idata", "lowpower", "stayawake", "debug-mode", "lowpower-timer" };
    for (unsigned i = 0; i < sizeof(removed) / sizeof(removed[0]); i++) {
        char line[32];
        strcpy(line, removed[i]);
        assert(!registered(line) && !shell_dispatch(line));
    }
    const char *retained[] = { "help", "status", "who", "fs-mount", "fs-unmount", "fs-ls",
        "settime", "rtc-settime", "rtc-time", "rtc-epoch", "rtc-temp", "rtc-timer-set",
        "rtc-timer-stop", "rtc-timer-status" };
    for (unsigned i = 0; i < sizeof(retained) / sizeof(retained[0]); i++) assert(registered(retained[i]));

    /* Invalid input must not trigger discovery or mutate settings. */
    invalid("licor extra");
    invalid("licor-query cfg<");
    invalid("licor-set cfg heater");
    invalid("licor-set cfg heater <TRUE>");
    invalid("licor-heater maybe");
    invalid("licor-model 830junk");
    invalid("licor-resync -1");
    invalid("licor-resync 5001");
    invalid("licor-resync 12junk");
    invalid("licor-set a b c d e f g h i");
    assert(li8x0_get_model() == LI8X0_UNKNOWN);

    command("licor");
    assert(requests == 2 && li8x0_get_model() == LI8X0_LI830);
    assert(strstr(output, "CO2: 412.2500 ppm") && strstr(output, "Raw CO2: 123"));
    assert(!strstr(output, "H2O") && !strstr(output, "nan"));
    command("licor-raw");
    assert(strstr(output, sample));
    command("licor-info");
    assert(strstr(output, "Serial: 83TEST") && strstr(output, "Firmware: 1.23"));
    command("licor-configure");
    assert(strstr(output, "configured") && requests == 14);
    command("licor-set rs232 raw co2 TRUE");
    assert(!strcmp(sent, "<li830><rs232><raw><co2>TRUE</co2></raw></rs232></li830>\r\n"));
    command("licor-heater off");
    assert(strstr(sent, "<heater>FALSE</heater>"));
    command("licor-query cal");
    assert(strstr(output, "<co2kzero>1.25</co2kzero>"));
    strcpy(long_reply, "<li830><cfg><diagnostic>");
    memset(long_reply + strlen(long_reply), 'X', 600);
    strcat(long_reply, "</diagnostic></cfg></li830>");
    command("licor-query cfg");
    assert(strstr(output, long_reply)); /* More than shell_printf's 256 bytes. */

    /* Explicit synchronization only; failed settings are never retried. */
    silence = true;
    command("licor-heater on");
    assert(strstr(output, "timeout") && g_app.licor_status == PERIPH_ERROR);
    unsigned before = requests;
    command("licor");
    assert(requests == before && strstr(output, "resynchronization required"));
    command("licor-model 830");
    assert(g_app.licor_status == PERIPH_ERROR);
    command("licor");
    assert(requests == before); /* Model selection cannot clear a sync fault. */
    silence = false;
    command("licor-resync 100");
    assert(requests == before && strstr(output, "synchronized"));
    command("licor");
    assert(requests == before + 1 && g_app.licor_status == PERIPH_READY);

    /* Oversize terminal input is discarded, not executed as a truncated setting. */
    shell_init();
    before = requests;
    output[0] = '\0';
    const char *prefix = "licor-heater on";
    while (*prefix) shell_process_char((uint8_t)*prefix++);
    for (unsigned i = 0; i < SHELL_MAX_CMD_LEN; i++) shell_process_char(' ');
    shell_process_char('\r');
    assert(requests == before && strstr(output, "Command too long; discarded"));
    puts("LI-COR shell integration tests passed");
    return 0;
}
