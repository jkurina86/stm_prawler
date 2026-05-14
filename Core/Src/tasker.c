/**
  ******************************************************************************
  * @file    tasker.c
  * @brief   Simplified task scheduler implementation
  * @note    Function pointer queue for deferred command execution
  ******************************************************************************
  */

#include "tasker.h"

/* Private types -------------------------------------------------------------*/

/**
 * @brief Queued task structure
 */
typedef struct {
    task_fn_t fn;                   /* Function to execute */
    uint8_t arg[TASK_ARG_SIZE];     /* Inline argument storage */
} queued_task_t;

/* Private variables ---------------------------------------------------------*/

/** Circular queue of pending tasks */
static queued_task_t task_queue[TASK_QUEUE_LEN];

/** Queue head (next write position) */
static volatile uint8_t queue_head = 0;

/** Queue tail (next read position) */
static volatile uint8_t queue_tail = 0;

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize the task scheduler
 */
void tasker_init(void)
{
    queue_head = 0;
    queue_tail = 0;
    memset(task_queue, 0, sizeof(task_queue));
}

/**
 * @brief Execute all pending tasks in queue order
 */
void tasker_run(void)
{
    while (queue_tail != queue_head) {
        queued_task_t *task = &task_queue[queue_tail];
        
        if (task->fn != NULL) {
            task->fn(task->arg);
        }
        
        /* Advance tail with wraparound (power of 2 optimization) */
        queue_tail = (queue_tail + 1) & (TASK_QUEUE_LEN - 1);
    }
}

/**
 * @brief Enqueue a task for deferred execution
 * @param fn      Function to call (must not be NULL)
 * @param arg     Pointer to argument data (copied into queue, may be NULL)
 * @param arg_len Size of argument data in bytes (0 if arg is NULL)
 * @retval true if successfully enqueued, false if queue full
 */
bool tasker_enqueue(task_fn_t fn, const void *arg, size_t arg_len)
{
    if (fn == NULL) {
        return false;
    }
    
    /* Calculate next head position */
    uint8_t next_head = (queue_head + 1) & (TASK_QUEUE_LEN - 1);
    
    /* Check if queue is full */
    if (next_head == queue_tail) {
        return false;
    }
    
    /* Get pointer to slot */
    queued_task_t *task = &task_queue[queue_head];
    
    /* Store function pointer */
    task->fn = fn;
    
    /* Copy argument data if provided */
    if (arg != NULL && arg_len > 0) {
        size_t copy_len = (arg_len < TASK_ARG_SIZE) ? arg_len : TASK_ARG_SIZE;
        memcpy(task->arg, arg, copy_len);
    } else {
        /* Clear argument storage */
        memset(task->arg, 0, TASK_ARG_SIZE);
    }
    
    /* Advance head */
    queue_head = next_head;
    
    return true;
}

/**
 * @brief Get number of pending tasks in queue
 * @retval Number of tasks waiting to be executed
 */
uint8_t tasker_pending_count(void)
{
    return (uint8_t)((queue_head - queue_tail) & (TASK_QUEUE_LEN - 1));
}
