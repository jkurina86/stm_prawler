/**
  ******************************************************************************
  * @file    shell.h
  * @brief   Simple shell interface
  ******************************************************************************
  */

#ifndef __SHELL_H
#define __SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    char *name;
    char *description;
    void (*function)(int argc, char **argv);
} shell_command_t;

/* Exported constants --------------------------------------------------------*/
#define SHELL_MAX_CMD_LEN       64
#define SHELL_MAX_ARGS          8
#define SHELL_PROMPT            "shell> "

/* Control Characters */
#define SHELL_CHAR_BS           0x08    /* Backspace */
#define SHELL_CHAR_TAB          0x09    /* Tab */
#define SHELL_CHAR_LF           0x0A    /* Line Feed */
#define SHELL_CHAR_CR           0x0D    /* Carriage Return */
#define SHELL_CHAR_ESC          0x1B    /* Escape */
#define SHELL_CHAR_DEL          0x7F    /* Delete */

/* Exported variables --------------------------------------------------------*/
extern const shell_command_t shell_commands[];

/* Exported function prototypes ----------------------------------------------*/
void shell_init(void);
void shell_process_char(uint8_t ch);
void shell_print(const char *str);
void shell_printf(const char *format, ...);
void shell_task(void);
bool shell_dispatch(char *cmd_line);
void shell_uart_receive_callback(void);

/* Command functions */
void cmd_help(int argc, char **argv);
void cmd_clear(int argc, char **argv);
void cmd_status(int argc, char **argv);
void cmd_pb8(int argc, char **argv);
void cmd_reset(int argc, char **argv);
void cmd_version(int argc, char **argv);
/* RTC Command functions */
void cmd_settime(int argc, char **argv);
void cmd_rtc_settime(int argc, char **argv);
void cmd_rtc_gettime(int argc, char **argv);
void cmd_rtc_temp(int argc, char **argv);
void cmd_rtc_timer_set(int argc, char **argv);
void cmd_rtc_timer_stop(int argc, char **argv);
void cmd_rtc_timer_status(int argc, char **argv);

/* CTD Command functions */
void cmd_ctd(int argc, char **argv);

/* Optode Command functions */
void cmd_optode(int argc, char **argv);
void cmd_optode_listen(int argc, char **argv);
void cmd_optode_setup(int argc, char **argv);

/* WetLab Command functions */
void cmd_wetlab(int argc, char **argv);
void cmd_wetlab_raw(int argc, char **argv);

/* Simultaneous sampling */
void cmd_sensors(int argc, char **argv);

/* Config */
void cmd_config(int argc, char **argv);

/* WiFi */
void cmd_wifi_status(int argc, char **argv);
void cmd_wifi_init(int argc, char **argv);

/* Realtime */
void cmd_realtime(int argc, char **argv);

/* File System Command functions */
void cmd_fs_mount(int argc, char **argv);
void cmd_fs_unmount(int argc, char **argv);
void cmd_fs_df(int argc, char **argv);
void cmd_fs_ls(int argc, char **argv);
void cmd_fs_cat(int argc, char **argv);
void cmd_fs_write(int argc, char **argv);
void cmd_fs_rm(int argc, char **argv);
void cmd_fs_mkdir(int argc, char **argv);
void cmd_fs_rmdir(int argc, char **argv);
void cmd_fs_cp(int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif /* __SHELL_H */
