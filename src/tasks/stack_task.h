//
// Created by Felix Hübner on 28.04.18.
//

#ifndef SAMC_FREERTOS_STACK_TASK_H
#define SAMC_FREERTOS_STACK_TASK_H

#include <asf.h>

/******** Defines ********/
#define STACK_TASK_MAX_TASKS 4

/******** Methods ********/
void task_wdt_reset(TaskHandle_t *task);

/**
 * will create the stack task, will store the given handles in global variables
 * @param handles task handles to test and enable
 * @param count total number of handles
 * @return NULL or on success a valid handle
 */
TaskHandle_t vCreateStackTask(TaskHandle_t *handles[], uint8_t count);

#endif //SAMC_FREERTOS_STACK_TASK_H
