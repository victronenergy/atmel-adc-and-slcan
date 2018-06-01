//
// Created by Felix Hübner on 28.04.18.
//

#ifndef SAMC_FREERTOS_STACK_TASK_H
#define SAMC_FREERTOS_STACK_TASK_H

#include <asf.h>

#define STACK_TASK_MAX_TASKS 4

/**
 *
 *
 *
 *
 */

/**
 * stacktask handler is used to control the running tasks + itself. (monitor there stack sizes)
 * @param handles array of TaskHandle_t, the first entry will be resumed first.
 * @param count number of TaskHandle_t in array
 * @return Handle of the StackTask
 */
TaskHandle_t vCreateStackTask(TaskHandle_t *handles[], uint8_t count);

#endif //SAMC_FREERTOS_STACK_TASK_H
