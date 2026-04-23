/**
  ******************************************************************************
  * @file    tasker.h
  * @brief   Simplified task scheduler using function pointer queue
  * @note    Tasks are queued as function pointers with inline argument storage
  ******************************************************************************
  */
#ifndef INC_TASKER_H_
#define INC_TASKER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Task function signature
 * @param arg Pointer to argument data (task-specific, may be NULL)
 */
typedef void (*task_fn_t)(const void *arg);

/* Exported constants --------------------------------------------------------*/

/** Maximum bytes for inline task argument storage */
#define TASK_ARG_SIZE 16

/** Task queue length (must be power of 2) */
#define TASK_QUEUE_LEN 16

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Initialize the task scheduler
 */
void tasker_init(void);

/**
 * @brief Execute all pending tasks in queue order
 * @note Call this from main loop
 */
void tasker_run(void);

/**
 * @brief Enqueue a task for deferred execution
 * @param fn      Function to call (must not be NULL)
 * @param arg     Pointer to argument data (copied into queue, may be NULL)
 * @param arg_len Size of argument data in bytes (0 if arg is NULL)
 * @retval true if successfully enqueued, false if queue full
 */
bool tasker_enqueue(task_fn_t fn, const void *arg, size_t arg_len);

/**
 * @brief Get number of pending tasks in queue
 * @retval Number of tasks waiting to be executed
 */
uint8_t tasker_pending_count(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASKER_H_ */
