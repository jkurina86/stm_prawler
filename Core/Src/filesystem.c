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
static FIL fil;                         /* File object */
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
  * @brief Get file system free space information
  * @param total_bytes: Pointer to store total space in bytes
  * @param free_bytes: Pointer to store free space in bytes  
  * @param used_percent: Pointer to store used space percentage
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_df(uint32_t *total_bytes, uint32_t *free_bytes, uint32_t *used_percent) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!total_bytes || !free_bytes || !used_percent) {
        return FS_INVALID_PARAM;
    }
    
    DWORD fre_clust;
    DWORD fre_sect; 
    DWORD tot_sect;
    FATFS *pfs;

    fs_result = f_getfree("0:", &fre_clust, &pfs);
    if (fs_result == FR_OK) {
        tot_sect = (pfs->n_fatent - 2) * pfs->csize;
        fre_sect = fre_clust * pfs->csize;

        *total_bytes = (uint32_t)tot_sect * 512UL;
        *free_bytes = (uint32_t)fre_sect * 512UL;
        
        if (tot_sect > 0) {
            *used_percent = ((tot_sect - fre_sect) * 100U) / tot_sect;
        } else {
            *used_percent = 0;
        }
        
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
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
  * @brief Create a directory
  * @param dirname: Directory name to create
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_mkdir(const char *dirname) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!dirname || strlen(dirname) == 0) {
        return FS_INVALID_PARAM;
    }
    
    fs_result = f_mkdir(dirname);
    if (fs_result == FR_OK) {
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Remove a directory
  * @param dirname: Directory name to remove
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_rmdir(const char *dirname) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!dirname || strlen(dirname) == 0) {
        return FS_INVALID_PARAM;
    }
    
    fs_result = f_unlink(dirname);
    if (fs_result == FR_OK) {
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
  * @brief Write text data to a file
  * @param filename: Name of file to write
  * @param data: Text data to write to file
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_write(const char *filename, const char *data) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!filename || strlen(filename) == 0 || !data) {
        return FS_INVALID_PARAM;
    }
    
    fs_result = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fs_result == FR_OK) {
        UINT bytes_written;
        fs_result = f_write(&fil, data, strlen(data), &bytes_written);
        f_close(&fil);
        
        if (fs_result == FR_OK) {
            return FS_OK;
        }
    }
    
    return convert_fatfs_result(fs_result);
}

/**
 * @brief Append text data to a file
 * @param filename: Name of file to append to
 * @param data: Text data to append to file
 * @retval FS_Result_t: Operation result
 */
FS_Result_t filesystem_append(const char *filename, const char *data) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }

    if (!filename || strlen(filename) == 0 || !data) {
        return FS_INVALID_PARAM;
    }

    fs_result = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fs_result == FR_OK) {
        UINT bytes_written;
        fs_result = f_write(&fil, data, strlen(data), &bytes_written);
        f_close(&fil);

        if (fs_result == FR_OK) {
            return FS_OK;
        }
    }

    return convert_fatfs_result(fs_result);
}

/**
  * @brief Delete a file
  * @param filename: Name of file to delete
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_rm(const char *filename) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!filename || strlen(filename) == 0) {
        return FS_INVALID_PARAM;
    }
    
    fs_result = f_unlink(filename);
    if (fs_result == FR_OK) {
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Copy a file
  * @param source: Source file name
  * @param destination: Destination file name
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_cp(const char *source, const char *destination) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!source || strlen(source) == 0 || !destination || strlen(destination) == 0) {
        return FS_INVALID_PARAM;
    }
    
    fs_result = f_open(&fil, source, FA_READ);
    if (fs_result == FR_OK) {
        FIL dest_fil;
        fs_result = f_open(&dest_fil, destination, FA_WRITE | FA_CREATE_ALWAYS);
        if (fs_result == FR_OK) {
            UINT bytes_read, bytes_written;
            do {
                fs_result = f_read(&fil, fs_buffers.file_data, sizeof(fs_buffers.file_data), &bytes_read);
                if (fs_result != FR_OK) break;
                if (bytes_read > 0) {
                    fs_result = f_write(&dest_fil, fs_buffers.file_data, bytes_read, &bytes_written);
                    if (fs_result != FR_OK || bytes_written < bytes_read) break;
                }
            } while (bytes_read > 0);
            f_close(&dest_fil);
        }
        f_close(&fil);
        
        if (fs_result == FR_OK) {
            return FS_OK;
        }
    }
    
    return convert_fatfs_result(fs_result);
}

/**
  * @brief Open the log file for appending
  * @param log_file: Pointer to FIL object for the log file
  * @param filename: Name of the log file to open
  * @retval FS_Result_t: Operation result
  */
FS_Result_t filesystem_open_log(FIL* log_file, const char* filename) {
    if (!fs_mounted) {
        return FS_NOT_MOUNTED;
    }
    
    if (!log_file || !filename) {
        return FS_INVALID_PARAM;
    }
    
    /* Open for writing, create if doesn't exist, append if exists */
    fs_result = f_open(log_file, filename, FA_WRITE | FA_OPEN_APPEND | FA_OPEN_ALWAYS);
    if (fs_result == FR_OK) {
        return FS_OK;
    }
    
    return convert_fatfs_result(fs_result);
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