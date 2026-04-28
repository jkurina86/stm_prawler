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
    FS_FILE_EXISTS,
    FS_ACCESS_DENIED
} FS_Result_t;

/* Buffers for shell integration */
typedef struct {
    char filename[64];
    char file_data[256];
} FS_Buffers_t;

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ---------------------------------------------*/

/* File system management */
FS_Result_t filesystem_init(void);
FS_Result_t filesystem_mount(void);
FS_Result_t filesystem_unmount(void);
void filesystem_force_reset(void);
bool filesystem_is_mounted(void);
int filesystem_last_fatfs_result(void);

/* Directory operations */
FS_Result_t filesystem_ls(void (*print_callback)(const char *));

/* File operations */
FS_Result_t filesystem_cat(const char *filename, void (*print_callback)(const char *));

/* Log file operations for recorder */
FS_Result_t filesystem_log_create(const char *filename);
FS_Result_t filesystem_log_write(const uint8_t *data, uint16_t len);
FS_Result_t filesystem_log_sync(void);
FS_Result_t filesystem_log_close(void);

/* File search operations */
FS_Result_t filesystem_find_latest(const char *suffix, char *out, int out_size);

/* Buffer management for shell integration */
FS_Buffers_t* filesystem_get_buffers(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_FILESYSTEM_H_ */
