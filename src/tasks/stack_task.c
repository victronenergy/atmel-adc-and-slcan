//
// Created by Felix Hübner on 28.04.18.
//

#include <log.h>
#include <stack_task.h>



void vStackTask(void *pvParameters);
void test_mark(UBaseType_t *rev, TaskHandle_t *task, bool override);

TaskHandle_t **task_handles;
uint8_t num_handles = 0;

void test_mark(UBaseType_t *rev, TaskHandle_t *task, bool override) {
	UBaseType_t i;

	i = uxTaskGetStackHighWaterMark(*task);
	if (i != *rev || override) {
		*rev = i;
		uint8_t s[16];
		uint8_t l = sprintf((char *) &s[0], "%lu", *rev);
		ulog_s(pcTaskGetName(*task));
		ulog_s(" mark: ");
		xlog(s, l);
//		ulog_s(" ");
//		l = sprintf((char *) &s[0], "%i %i %i", ((uint8_t)*rev), ((uint8_t)*(rev+1)), ((uint8_t)*(rev+2)));
//		xlog(s, l);

		ulog_s("\r\n");
	}
}


void vStackTask(void *pvParameters) {
	ulog_s("stacktask begin loop\r\n");

	// check if the freemem array will be big enough to hold all task values
	if (num_handles > STACK_TASK_MAX_TASKS) {
		ulog_s("ERROR: Increase STACK_TASK_MAX_TASKS in stack_task.h to 0x");
		xlog(&num_handles,1);
		while (1) {};
	}

	UBaseType_t freemem[STACK_TASK_MAX_TASKS+1] = {0};

	TaskHandle_t s = NULL;

	vTaskDelay((const TickType_t) 100);

	for (uint8_t i = 0; i<num_handles; i++) {
		vTaskResume((*task_handles)[i]);
		vTaskDelay((const TickType_t) 100);
	}

	for (;;) {
		const bool override = false;
		for (uint8_t i = 0; i<num_handles; i++) {
			test_mark( &freemem[i], (*task_handles)[i], override);
		}
		test_mark( &freemem[num_handles+1], &s, override);
		vTaskDelay((const TickType_t) 1000);
	}
}




TaskHandle_t vCreateStackTask(TaskHandle_t *handles, uint8_t count) {
	*task_handles = handles;
	num_handles = count;

	ulog_s("creating Stack Task...\r\n");

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vStackTask,       /* Function that implements the task. */
			"sTASK",          /* Text name for the task. */
			250,      /* Stack size in words, not bytes. */
			NULL,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY + 5,/* Priority at which the task is created. */
			&xHandle);      /* Used to pass out the created task's handle. */

	if( xReturned == pdPASS )
	{
		ulog_s("\r\nsuccessfully created Stack Task\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		//vTaskDelete( xHandle );
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;
}