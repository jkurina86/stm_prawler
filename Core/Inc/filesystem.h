/**
  ******************************************************************************
  * @file    filesystem.h
  * @brief   Driver for FatFS file system operations in the shell environment.
  ******************************************************************************
  */
#ifndef INC_FILESYSTEM_H_
#define INC_FILESYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ff.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* File system operation results */
typedef enum {
    FS_OK = 0,
    FS_ERROR,
    FS_NOT_MOUNTED,
    FS_ALREADY_MOUNTED,
    FS_INVALID_PARAM,
    FS_FILE_NOT_FOUND,
    FS_ACCESS_DENIED
} FS_Result_t;

/* Buffers for shell integration */
typedef struct {
    char filename[64];
    char dest_filename[64];
    char dirname[128];
    char file_data[256];
} FS_Buffers_t;

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ---------------------------------------------*/

/* File system management */
FS_Result_t filesystem_init(void);
FS_Result_t filesystem_mount(void);
FS_Result_t filesystem_unmount(void);
bool filesystem_is_mounted(void);

/* File system information */
FS_Result_t filesystem_df(uint32_t *total_bytes, uint32_t *free_bytes, uint32_t *used_percent);

/* Directory operations */
FS_Result_t filesystem_ls(void (*print_callback)(const char *));
FS_Result_t filesystem_mkdir(const char *dirname);
FS_Result_t filesystem_rmdir(const char *dirname);

/* File operations */
FS_Result_t filesystem_cat(const char *filename, void (*print_callback)(const char *));
FS_Result_t filesystem_write(const char *filename, const char *data);
FS_Result_t filesystem_append(const char *filename, const char *data);
FS_Result_t filesystem_rm(const char *filename);
FS_Result_t filesystem_cp(const char *source, const char *destination);

/* Log file operations for recorder */
FS_Result_t filesystem_open_log(FIL* log_file, const char* filename);

/* Buffer management for shell integration */
FS_Buffers_t* filesystem_get_buffers(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_FILESYSTEM_H_ */