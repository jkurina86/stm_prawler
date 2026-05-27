/**
  ******************************************************************************
  * @file    task_handlers.h
  * @brief   Deferred task handler prototypes and argument structures
  ******************************************************************************
  */

#ifndef INC_TASK_HANDLERS_H_
#define INC_TASK_HANDLERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Argument structures ---------------------------------------------------*/

typedef struct {
    uint16_t year;
    uint8_t months;
    uint8_t days;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t valid;
} rtc_settime_args_t;

typedef struct {
    uint32_t unix_epoch;
} settime_args_t;

typedef struct {
    uint16_t seconds;
} rtc_timer_set_args_t;

typedef struct {
    uint8_t level;
    uint8_t set;    /* 0 = get, 1 = set */
} config_args_t;

/* Handler prototypes (all void fn(const void *arg)) ---------------------*/

/* General */
void handle_help(const void *arg);
void handle_clear(const void *arg);
void handle_status(const void *arg);
void handle_lowpower(const void *arg);
void handle_stayawake(const void *arg);
void handle_debug_mode(const void *arg);
void handle_lowpower_timer(const void *arg);
void handle_version(const void *arg);
/* RTC */
void handle_settime(const void *arg);
void handle_rtc_settime(const void *arg);
void handle_rtc_gettime(const void *arg);
void handle_rtc_epoch(const void *arg);
void handle_rtc_temp(const void *arg);
void handle_rtc_timer_set(const void *arg);
void handle_rtc_timer_stop(const void *arg);
void handle_rtc_timer_status(const void *arg);

/* CTD */
void handle_ctd(const void *arg);
void handle_ctd_raw(const void *arg);

/* Optode */
void handle_optode(const void *arg);
void handle_optode_raw(const void *arg);
void handle_optode_listen(const void *arg);

/* WetLab */
void handle_wetlab(const void *arg);
void handle_wetlab_raw(const void *arg);

/* Simultaneous sampling */
void handle_sensors(const void *arg);

/* Config */
void handle_config(const void *arg);
void handle_samplerate(const void *arg);

/* WiFi */
void handle_wifi_status(const void *arg);
void handle_wifi_up(const void *arg);
void handle_wifi_down(const void *arg);

/* Realtime */
void handle_idata(const void *arg);
void handle_who(const void *arg);

/* File System */
void handle_fs_mount(const void *arg);
void handle_fs_unmount(const void *arg);
void handle_fs_ls(const void *arg);

#ifdef __cplusplus
}
#endif

#endif /* INC_TASK_HANDLERS_H_ */
