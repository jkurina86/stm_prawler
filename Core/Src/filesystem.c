/**
  ******************************************************************************
  * @file    filesystem.c
  * @brief   Driver for FatFS file system operations in the shell environment.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "filesystem.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static FATFS fs;                        /* File system object */
static FIL fil;                         /* File object for shell commands */
static FIL log_fil;                     /* File object for recorder log */
static char fs_path[128] = "0:";        /* Current file system path */
static FRESULT fs_result;               /* File operation result */
static volatile uint8_t fs_mounted = 0; /* File system mounted flag */
static FS_Buffers_t fs_buffers;         /* Buffers for shell integration */

/* Private function prototypes -----------------------------------------------*/
static FS_Result_t convert_fatfs_result(FRESULT result);

/* Public functions ----------------------------------------------------------*/

/**
  * @brief Initialize the filesystem module
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_init(void) {
    /* Clear buffers */
    memset(&fs_buffers, 0, sizeof(fs_buffers));
    
    /* Initialize path */
    strcpy(fs_path, "0:");
    
    fs_mounted = 0;
    
    return FS_OK;
}

/**
  * @brief Mount the file system
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_mount(void) {
    if (fs_mounted) {
        return FS_ALREADY_MOUNTED;
    }
    
    fs_result = f_mount(&fs, "0:", 1);
    if (fs_result == FR_OK) {
        fs_mounted = 1;
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Unmount the file system
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_unmount(void) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    fs_result = f_mount(NULL, "0:", 0);
    if (fs_result == FR_OK) {
        fs_mounted = 0;
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Check if filesystem is mounted
  * @retval bool: true if mounted, false otherwise
  */
bool filesystem_is_mounted(void) {
    return fs_mounted != 0;
}

/**
  * @brief List directory contents
  * @param print_callback: Function to call for each directory entry
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_ls(void (*print_callback)(const char *)) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!print_callback) {
        return FS_INVALID_PARAM;
    }
    
    DIR dir;
    FILINFO fno;
    char line_buffer[300];
    
    fs_result = f_opendir(&dir, fs_path);
    if (fs_result == FR_OK) {
        fs_result = f_readdir(&dir, &fno);
        while (fs_result == FR_OK && fno.fname[0] != 0) {
            if (fno.fattrib & AM_DIR) {
                snprintf(line_buffer, sizeof(line_buffer), "<DIR>  %.255s\r\n", fno.fname);
            } else {
                snprintf(line_buffer, sizeof(line_buffer), "       %.255s %8lu Bytes\r\n", fno.fname, fno.fsize);
            }
            print_callback(line_buffer);
            fs_result = f_readdir(&dir, &fno);
        }
        f_closedir(&dir);
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Read and print file contents
  * @param filename: Name of file to read
  * @param print_callback: Function to call for file content output
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_cat(const char *filename, void (*print_callback)(const char *)) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!filename || strlen(filename) == 0 || !print_callback) {
        return FS_INVALID_PARAM;
    }
    
    fs_result = f_open(&fil, filename, FA_READ);
    if (fs_result == FR_OK) {
        UINT bytes_read;
        while (f_read(&fil, fs_buffers.file_data, sizeof(fs_buffers.file_data) - 1, &bytes_read) == FR_OK && bytes_read > 0) {
            fs_buffers.file_data[bytes_read] = '\0';
            print_callback(fs_buffers.file_data);
        }
        f_close(&fil);
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Create a new log file (fails if file already exists)
  * @param filename: Name of the log file to create
  * @retval FS_Result_t: FS_OK, FS_FILE_EXISTS, or error
  */
FS_Result_t filesystem_log_create(const char *filename) {
    if (!fs_mounted) return FS_NOT_MOUNTED;
    if (!filename)   return FS_INVALID_PARAM;

    fs_result = f_open(&log_fil, filename, FA_WRITE | FA_CREATE_NEW);
    if (fs_result == FR_OK)    return FS_OK;
    if (fs_result == FR_EXIST) return FS_FILE_EXISTS;
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Write data to the open log file
  * @param data: Buffer to write
  * @param len:  Number of bytes to write
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_log_write(const uint8_t *data, uint16_t len) {
    UINT bw;
    fs_result = f_write(&log_fil, data, len, &bw);
    if (fs_result != FR_OK || bw != len) return FS_ERROR;
    return FS_OK;
}

/**
  * @brief Sync (flush) the open log file to disk
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_log_sync(void) {
    fs_result = f_sync(&log_fil);
    return (fs_result == FR_OK) ? FS_OK : FS_ERROR;
}

/**
  * @brief Close the open log file
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_log_close(void) {
    fs_result = f_close(&log_fil);
    return (fs_result == FR_OK) ? FS_OK : FS_ERROR;
}

/**
  * @brief Find the file with the latest modification date matching a suffix
  * @param suffix: Filename suffix to match (e.g., "_record.csv")
  * @param out: Buffer to receive the filename
  * @param out_size: Size of the output buffer
  * @retval FS_OK if a matching file was found, FS_FILE_NOT_FOUND otherwise
  */
FS_Result_t filesystem_find_latest(const char *suffix, char *out, int out_size) {
    if (!fs_mounted) return FS_NOT_MOUNTED;
    if (!suffix || !out || out_size <= 0) return FS_INVALID_PARAM;

    DIR dir;
    FILINFO fno;
    WORD best_date = 0;
    WORD best_time = 0;
    bool found = false;

    fs_result = f_opendir(&dir, fs_path);
    if (fs_result != FR_OK)
        return convert_fatfs_result(fs_result);

    int suffix_len = strlen(suffix);

    fs_result = f_readdir(&dir, &fno);
    while (fs_result == FR_OK && fno.fname[0] != 0) {
        if (!(fno.fattrib & AM_DIR)) {
            int name_len = strlen(fno.fname);
            if (name_len >= suffix_len &&
                strcmp(fno.fname + name_len - suffix_len, suffix) == 0) {
                /* Compare FAT date/time (higher = newer) */
                if (fno.fdate > best_date ||
                    (fno.fdate == best_date && fno.ftime > best_time)) {
                    best_date = fno.fdate;
                    best_time = fno.ftime;
                    strncpy(out, fno.fname, out_size - 1);
                    out[out_size - 1] = '\0';
                    found = true;
                }
            }
        }
        fs_result = f_readdir(&dir, &fno);
    }

    f_closedir(&dir);
    return found ? FS_OK : FS_FILE_NOT_FOUND;
}

/**
  * @brief Get filesystem buffers for shell integration
  * @retval FS_Buffers_t*: Pointer to filesystem buffers
  */
FS_Buffers_t* filesystem_get_buffers(void) {
    return &fs_buffers;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Convert FatFS result to filesystem result
  * @param result: FatFS result code
  * @retval FS_Result_t: Converted result
  * @note This is done to avoid exposing FatFS types outside this module
  */
static FS_Result_t convert_fatfs_result(FRESULT result) {
    switch (result) {
        case FR_OK:
            return FS_OK;
        case FR_NO_FILE:
        case FR_NO_PATH:
            return FS_FILE_NOT_FOUND;
        case FR_DENIED:
        case FR_WRITE_PROTECTED:
            return FS_ACCESS_DENIED;
        default:
            return FS_ERROR;
    }
}
