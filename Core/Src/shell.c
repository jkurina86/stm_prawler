/**
  ******************************************************************************
  * @file    shell.c
  * @brief   Terminal Command Interface
  * @note    Simple shell interface over UART with command parsing and tasking
  * @note    DO NOT print or call blocking functions from within the UART RX ISR
  ******************************************************************************
  */

#include "shell.h"
#include "main.h"
#include "tasker.h"
#include "task_handlers.h"
#include "config.h"
#include "filesystem.h"
#include "wifi.h"
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

/* Private variables ---------------------------------------------------------*/
static char shell_buffer[SHELL_MAX_CMD_LEN];
static uint16_t shell_buffer_pos = 0;
static uint8_t shell_rx_char;

/* Non-blocking ISR to main RX ring buffer */
#define SHELL_RX_RING_SIZE 128
static volatile uint8_t shell_rx_ring[SHELL_RX_RING_SIZE];
static volatile uint16_t shell_rx_head = 0;
static volatile uint16_t shell_rx_tail = 0;

/**
 * @brief Enqueue received character into ring buffer (non-blocking)
 * @param c Character to enqueue
 * @note Drops character silently if buffer is full
 */
static inline void shell_rx_enqueue(uint8_t c)
{
    /* Find next position in the ring buffer */
    uint16_t next_head = (shell_rx_head + 1) % SHELL_RX_RING_SIZE;
    if (next_head == shell_rx_tail) {
        /* Buffer full: drop the character (non-blocking) */
        return;
    }
    shell_rx_ring[shell_rx_head] = c;
    shell_rx_head = next_head;
}

/* Command table */
const shell_command_t shell_commands[] = {
    /* General Commands */
    {"help", "Display available commands", cmd_help},
    {"clear", "Clear terminal screen", cmd_clear},
    {"status", "Show system status", cmd_status},
    {"reset", "Reset the system", cmd_reset},
    {"version", "Show firmware version", cmd_version},

    /* RTC Commands */
    {"rtc-settime", "Set RTC date/time (YYYY MM DD HH MM SS)", cmd_rtc_settime},
    {"rtc-time", "Get current RTC date/time", cmd_rtc_gettime},
    {"rtc-temp", "Get RTC temperature", cmd_rtc_temp},
    {"rtc-timer-set", "Set RTC timer (seconds)", cmd_rtc_timer_set},
    {"rtc-timer-stop", "Stop RTC timer", cmd_rtc_timer_stop},
    {"rtc-timer-status", "Show RTC timer status", cmd_rtc_timer_status},

    /* CTD Commands */
    {"ctd", "Get CTD configuration data", cmd_ctd},

    /* Optode Commands */
    {"optode", "Get Optode sensor data", cmd_optode},
    {"optode-listen", "Power-cycle optode and listen", cmd_optode_listen},
    {"optode-setup", "Set optode to 9600 baud + terminal mode", cmd_optode_setup},

    /* WetLab Commands */
    {"wetlab", "Get WetLab sensor data", cmd_wetlab},
    {"wetlab-raw", "Dump raw WetLab output (4s)", cmd_wetlab_raw},

    /* Simultaneous Sampling */
    {"sensors", "Sample all sensors simultaneously", cmd_sensors},

    /* Config */
    {"config", "Get/set sensor config (1-3)", cmd_config},

    /* File System Commands */
    {"fs-mount", "Mount the file system", cmd_fs_mount},
    {"fs-unmount", "Unmount the file system", cmd_fs_unmount},
    {"fs-df", "Show file system free space", cmd_fs_df},
    {"fs-ls", "List directory contents", cmd_fs_ls},
    {"fs-cat", "Read a file", cmd_fs_cat},
    {"fs-write", "Write a file", cmd_fs_write},
    {"fs-rm", "Delete a file", cmd_fs_rm},
    {"fs-mkdir", "Create a directory", cmd_fs_mkdir},
    {"fs-rmdir", "Remove a directory", cmd_fs_rmdir},
    {"fs-cp", "Copy a file", cmd_fs_cp},

    /* WiFi Commands */
    {"wifi-status", "Show WiFi module state", cmd_wifi_status},
    {"wifi-send", "Send message over WiFi", cmd_wifi_send},
    {"wifi-poll", "Poll for incoming WiFi message", cmd_wifi_poll},
    {"wifi-reset", "Reset WiFi module and reinitialize", cmd_wifi_reset},
    {"wifi-dump", "Dump raw WiFi ring buffer contents", cmd_wifi_dump},

    {NULL, NULL, NULL} /* End marker */
};

/* Private function prototypes -----------------------------------------------*/
static void shell_execute_command(char *cmd_line);
static int shell_parse_command(char *cmd_line, char **argv);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief Initialize the shell
  * @param None
  * @retval None
  * @note Call this from main() during system initialization.
  */
void shell_init(void)
{
    /* Clear buffers */
    memset(shell_buffer, 0, sizeof(shell_buffer));
    shell_buffer_pos = 0;

    /* Start UART reception in interrupt-mode (HAL) */
    HAL_UART_Receive_IT(&huart1, &shell_rx_char, 1);

    /* Print welcome message */
    shell_print("\n===============================================\n");
    shell_print("       System Shell\n");
    shell_print("===============================================\n");
    shell_print("Type 'help' for available commands\r\n");
    shell_print(SHELL_PROMPT);
}

/**
  * @brief Process received characters
  * @param ch: Character to process
  * @retval None
  * @details Note that this occurs outside of the ISR. It's called by shell_task() from main().
  */
void shell_process_char(uint8_t ch)
{
    switch (ch) {
    	/* Enter/Return */
        case SHELL_CHAR_CR:
        case SHELL_CHAR_LF:
            shell_print("\r\n");
            /* Execute command and reset the buffer if there's something contained in the buffer. */
            if (shell_buffer_pos > 0) {
                shell_buffer[shell_buffer_pos] = '\0'; /* Null-terminate the string. */
                shell_execute_command(shell_buffer);
                shell_buffer_pos = 0;
                memset(shell_buffer, 0, sizeof(shell_buffer)); /* Zeros the char buffer in memory. */
            }
            shell_print(SHELL_PROMPT);
            break;

        /* Backspace/Delete */
        case SHELL_CHAR_BS:
        case SHELL_CHAR_DEL:
            /* If the buffer isn't empty, move the buffer position back one and null-terminate. */
            if (shell_buffer_pos > 0) {
                shell_buffer_pos--;
                shell_buffer[shell_buffer_pos] = '\0';
                shell_print("\b \b"); /* Backspace, overwrite with a space, backspace */
            }
            break;

        case SHELL_CHAR_TAB:
            break; /* Ignore TAB */

        case SHELL_CHAR_ESC:
            break; /* Ignore ESC */

        default:
            /* Regular character */
            if (ch >= 32 && ch <= 126 /* ASCII chars, from space=32 to ~=126 */
            		&& shell_buffer_pos < (SHELL_MAX_CMD_LEN - 1)) {
                /* Put the received ASCII char into the shell buffer */
            	shell_buffer[shell_buffer_pos++] = ch;
                /* Echo character. */
                HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
            }
            break;
    }
}

/**
  * @brief Print string to shell
  * @param str: String to print
  * @retval None
  * @note This is a simple blocking print function. Only call from outside of the ISR.
  */
void shell_print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief Printf wrapper for the shell
  * @param format: Format string
  * @param ...: Arguments
  * @retval None
  * @note This is a wrapper for printf and only called from outside of the ISR, so variadic arguments are fine.
  */
void shell_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args); /* Variadic format string */
    va_end(args);
    shell_print(buffer);
}

/**
  * @brief Shell background task to handle RX buffer processing.
  * @param None
  * @retval None
  * @note Call this from main loop. Drains RX ring buffer and processes input.
  */
void shell_task(void)
{
    /* Drain RX ring buffer and process characters in main context (non-ISR) */
    while (shell_rx_tail != shell_rx_head) {
        uint8_t ch = shell_rx_ring[shell_rx_tail];
        shell_rx_tail = (uint16_t)((shell_rx_tail + 1u) % SHELL_RX_RING_SIZE);
        shell_process_char(ch);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Execute parsed command
  * @param cmd_line: Command line to execute
  * @retval None
  * @note This is called from shell_process_char() outside of the ISR.
  */
static void shell_execute_command(char *cmd_line)
{
    char *argv[SHELL_MAX_ARGS];
    int argc = shell_parse_command(cmd_line, argv);

    if (argc == 0) {
        return;
    }

    /* Find and execute command */
    for (int i = 0; shell_commands[i].name != NULL; i++) {
        if (strcmp(argv[0], shell_commands[i].name) == 0) {
            shell_commands[i].function(argc, argv);
            return;
        }
    }

    /* Command not found */
    shell_printf("Unknown command: %s\r\n", argv[0]);
    shell_print("Type 'help' for available commands\r\n");
}

/**
  * @brief Parse command line into arguments
  * @param cmd_line: Command line to parse
  * @param argv: Argument array to fill
  * @retval Number of arguments
  * @note This is called from shell_execute_command() outside of the ISR.
  */
static int shell_parse_command(char *cmd_line, char **argv)
{
    int argc = 0;
    char *token = strtok(cmd_line, " ");

    while (token != NULL && argc < SHELL_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    return argc;
}

/* General Commands ------------------------------------------*/

/**
  * @brief Help command to list available commands
  * @note Schedules help task
  */
void cmd_help(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_help, NULL, 0);
}

/**
  * @brief Clear screen command
  * @note Schedules clear terminal task
  */
void cmd_clear(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_clear, NULL, 0);
}

/**
  * @brief System status command
  * @note Schedules status task
  */
void cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_status, NULL, 0);
}

/**
  * @brief Reset system command
  * @note Schedules a reset task with a 3-second delay
  */
void cmd_reset(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */

    /* Schedule reset task with 3 second delay */
    reset_args_t args = { .reset_due_ms = HAL_GetTick() + 3000u };
    tasker_enqueue(handle_reset, &args, sizeof(args));
}

/**
  * @brief Version command
  * @note  Schedules version task for deferred execution
  */
void cmd_version(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_version, NULL, 0);
}

/* RTC Commands -----------------------------------------------*/

/**
  * @brief Set RTC date and time
  * @param argc: Argument count
  * @param argv: Arguments (YYYY MM DD HH MM SS WD)
  * @retval None
  */
void cmd_rtc_settime(int argc, char **argv)
{
    /* Parse arguments and schedule RTC settime task */
    rtc_settime_args_t args = {0};
    
    if (argc >= 8 && argv != NULL) {
        args.year = (uint16_t)atoi(argv[1]);
        args.months = (uint8_t)atoi(argv[2]);
        args.days = (uint8_t)atoi(argv[3]);
        args.hours = (uint8_t)atoi(argv[4]);
        args.minutes = (uint8_t)atoi(argv[5]);
        args.seconds = (uint8_t)atoi(argv[6]);
        args.weekdays = (uint8_t)atoi(argv[7]);
        args.valid = 1;
    }
    
    tasker_enqueue(handle_rtc_settime, &args, sizeof(args));
}

/**
  * @brief Schedule an RTC get time task
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  */
void cmd_rtc_gettime(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_rtc_gettime, NULL, 0);
}

/**
  * @brief Schedule an RTC temperature read task
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  */
void cmd_rtc_temp(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_rtc_temp, NULL, 0);
}

/**
  * @brief Set RTC countdown timer
  * @param argc: Argument count
  * @param argv: Arguments (seconds)
  * @retval None
  */
void cmd_rtc_timer_set(int argc, char **argv)
{
    rtc_timer_set_args_t args = {0};

    if (argc >= 2) {
        args.seconds = (uint16_t)atoi(argv[1]);
    }

    tasker_enqueue(handle_rtc_timer_set, &args, sizeof(args));
}

/**
  * @brief Stop RTC countdown timer
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  */
void cmd_rtc_timer_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_rtc_timer_stop, NULL, 0);
}

/**
  * @brief Show RTC timer status
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  */
void cmd_rtc_timer_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_rtc_timer_status, NULL, 0);
}

/* File System Commands -----------------------------------------------*/

/** @brief Schedule a mount task
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  * @note Sets a flag to mount in the main loop context to avoid blocking in ISR
 */
void cmd_fs_mount(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */

    tasker_enqueue(handle_fs_mount, NULL, 0);
}

/** @brief Schedule an unmount task
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  * @note Sets a flag to unmount in the main loop context to avoid blocking in ISR
 */
void cmd_fs_unmount(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */

    tasker_enqueue(handle_fs_unmount, NULL, 0);
}

/**
  * @brief Schedule a df task
  * @param argc: Argument count
  * @param argv: Arguments
  * @retval None
  * @note Requires the file system to be mounted. Sets a flag to handle in main loop context.
  */
void cmd_fs_df(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */

    tasker_enqueue(handle_fs_df, NULL, 0);
}

/**
  * @brief Schedule an ls task
  * @param argc: Argument count
  * @param argv: Arguments (optional path)
  * @retval None
  */
void cmd_fs_ls(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    tasker_enqueue(handle_fs_ls, NULL, 0);
}

/**
  * @brief Schedule a cat task
  * @param argc: Argument count
  * @param argv: Arguments (file path)
  * @retval None
  */
void cmd_fs_cat(int argc, char **argv)
{
    (void)argc; /* Unused arg */
    /* Create a pointer to the file system buffers */
    FS_Buffers_t *buffers = filesystem_get_buffers();

    /* Copy filename into buffer */
    if (argv[1] != NULL) {
        strncpy(buffers->filename, argv[1], sizeof(buffers->filename) - 1);
        buffers->filename[sizeof(buffers->filename) - 1] = '\0'; /* Ensure null-termination */
    }

    tasker_enqueue(handle_fs_cat, NULL, 0);
}

/**
  * @brief Schedule a write task
  * @param argc: Argument count
  * @param argv: Arguments (file path, content)
  * @retval None
  */
void cmd_fs_write(int argc, char **argv)
{
    (void)argc; /* Unused arg */
    /* Create a pointer to the file system buffers */
    FS_Buffers_t *buffers = filesystem_get_buffers();

    /* Copy filename into buffer */
    if (argv[1] != NULL) {
        strncpy(buffers->filename, argv[1], sizeof(buffers->filename) - 1);
        buffers->filename[sizeof(buffers->filename) - 1] = '\0';
    }

    /* Copy file data into buffer */
    if (argv[2] != NULL) {
        strncpy(buffers->file_data, argv[2], sizeof(buffers->file_data) - 1);
        buffers->file_data[sizeof(buffers->file_data) - 1] = '\0';
    }

    tasker_enqueue(handle_fs_write, NULL, 0);
}

/**
  * @brief Schedule an rm task
  * @param argc: Argument count
  * @param argv: Arguments (file path)
  * @retval None
  */
void cmd_fs_rm(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    /* Create a pointer to the file system buffers */
    FS_Buffers_t *buffers = filesystem_get_buffers();

    /* Copy filename into buffer */
    if (argv[1] != NULL) {
        strncpy(buffers->filename, argv[1], sizeof(buffers->filename) - 1);
        buffers->filename[sizeof(buffers->filename) - 1] = '\0';
    }

    tasker_enqueue(handle_fs_rm, NULL, 0);
}

/**
  * @brief Schedule a mkdir task
  * @param argc: Argument count
  * @param argv: Arguments (directory path)
  * @retval None
  * @note Called from ISR context
  */
void cmd_fs_mkdir(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    /* Create a pointer to the file system buffers */
    FS_Buffers_t *buffers = filesystem_get_buffers();

    /* Copy directory name into buffer */
    if (argv[1] != NULL) {
        strncpy(buffers->dirname, argv[1], sizeof(buffers->dirname) - 1);
        buffers->dirname[sizeof(buffers->dirname) - 1] = '\0';
    }

    tasker_enqueue(handle_fs_mkdir, NULL, 0);
}

/**
  * @brief Schedule an rmdir task
  * @param argc: Argument count
  * @param argv: Arguments (directory path)
  * @retval None
  */
void cmd_fs_rmdir(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    /* Create a pointer to the file system buffers */
    FS_Buffers_t *buffers = filesystem_get_buffers();

    /* Copy directory name into buffer */
    if (argv[1] != NULL) {
        strncpy(buffers->dirname, argv[1], sizeof(buffers->dirname) - 1);
        buffers->dirname[sizeof(buffers->dirname) - 1] = '\0';
    }

    tasker_enqueue(handle_fs_rmdir, NULL, 0);
}

/** @brief Schedule a cp task
  * @param argc: Argument count
  * @param argv: Arguments (source path, destination path)
  * @retval None
  */
void cmd_fs_cp(int argc, char **argv)
{
    (void)argc; (void)argv; /* Unused args */
    /* Create a pointer to the file system buffers */
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    /* Copy source filename into buffer */
    if (argv[1] != NULL) {
        strncpy(buffers->filename, argv[1], sizeof(buffers->filename) - 1);
        buffers->filename[sizeof(buffers->filename) - 1] = '\0';
    }

    /* Copy destination filename into buffer */
    if (argv[2] != NULL) {
        strncpy(buffers->dest_filename, argv[2], sizeof(buffers->dest_filename) - 1);
        buffers->dest_filename[sizeof(buffers->dest_filename) - 1] = '\0';
    }

    tasker_enqueue(handle_fs_cp, NULL, 0);
}

/* CTD Commands ---------------------------------------------------*/

void cmd_ctd(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_ctd, NULL, 0);
}

/* Optode Commands ------------------------------------------------*/

void cmd_optode(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_optode, NULL, 0);
}

void cmd_optode_listen(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_optode_listen, NULL, 0);
}

void cmd_optode_setup(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_optode_setup, NULL, 0);
}

/* WetLab Commands ------------------------------------------------*/

void cmd_wetlab(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_wetlab, NULL, 0);
}

void cmd_wetlab_raw(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_wetlab_raw, NULL, 0);
}

/* Simultaneous Sampling ------------------------------------------*/

void cmd_sensors(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_sensors, NULL, 0);
}

/* Config Commands ------------------------------------------------*/

void cmd_config(int argc, char **argv)
{
    config_args_t args = {0};

    if (argc >= 2) {
        args.level = (uint8_t)atoi(argv[1]);
        args.set = 1;
    }

    tasker_enqueue(handle_config, &args, sizeof(args));
}

/* WiFi Commands ---------------------------------------------------*/

void cmd_wifi_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_wifi_status, NULL, 0);
}

void cmd_wifi_send(int argc, char **argv)
{
    if (argc < 2)
        return;

    char *buf = wifi_get_send_buf();
    buf[0] = '\0';

    /* Concatenate all args into the send buffer */
    uint16_t pos = 0;
    for (int i = 1; i < argc && pos < 255; i++) {
        if (i > 1 && pos < 255)
            buf[pos++] = ' ';
        uint16_t len = strlen(argv[i]);
        if (pos + len > 255)
            len = 255 - pos;
        memcpy(&buf[pos], argv[i], len);
        pos += len;
    }
    buf[pos] = '\0';

    tasker_enqueue(handle_wifi_send, NULL, 0);
}

void cmd_wifi_poll(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_wifi_poll, NULL, 0);
}

void cmd_wifi_reset(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_wifi_reset, NULL, 0);
}

void cmd_wifi_dump(int argc, char **argv)
{
    (void)argc; (void)argv;
    tasker_enqueue(handle_wifi_dump, NULL, 0);
}

/* UART Interrupt Callbacks -------------------------------------------------*/

/**
  * @brief Handle shell UART receive (called from main HAL_UART_RxCpltCallback)
  * @param None
  * @retval None
  */
void shell_uart_receive_callback(void)
{
    /* Enqueue received byte and do all parsing/printing in main with shell_task() */
    shell_rx_enqueue(shell_rx_char);
    HAL_UART_Receive_IT(&huart1, &shell_rx_char, 1);
}
