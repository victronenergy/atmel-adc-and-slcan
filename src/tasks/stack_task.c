//
// Created by Felix Hübner on 28.04.18.
//

#include <log.h>
#include <stack_task.h>


/******** Internal Prototypes ********/
void vStackTask(void *pvParameters);
void test_mark(UBaseType_t *rev, TaskHandle_t *task, bool override);


/******** Global Variables ********/
TaskHandle_t **task_handles;
uint8_t num_handles = 0;


/******** Methods ********/
/**
 * will test the given task if the free mem region has changed
 *
 * @param rev pointer to a variable that holds the last calculated value of free mem
 * @param task task handle to test
 * @param override override the test and output the current status
 */
void test_mark(UBaseType_t *rev, TaskHandle_t *task, bool override) {
	UBaseType_t i;

	i = uxTaskGetStackHighWaterMark(*task);
	if (i != *rev || override) {
		*rev = i;
		uint8_t s[16];
		uint8_t l = (uint8_t) sprintf((char *) &s[0], "%lu", *rev);
		ulog_s(pcTaskGetName(*task));
		ulog_s(" mark: ");
		xlog(s, l);
		ulog_s("\r\n");

	}
}


/**
 * stack task, will be active every 2 sec and will check if the memory regions of the other tasks have changed.
 * If the region has changed, the current free ram will pushed to debug output.
 * this task will be used to enable all other tasks in the given sequence
 * @param pvParameters not used
 */
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
		vTaskResume(*(task_handles[i]));
		vTaskDelay((const TickType_t) 100);
	}
	for (;;) {
		const bool override = false;
		for (uint8_t i = 0; i<num_handles; i++) {
			test_mark( &freemem[i], &(*(task_handles[i])), override);
		}
		test_mark( &freemem[num_handles+1], &s, override);
		vTaskDelay((const TickType_t) 2000);

		//reset the microcontroller internal watchdog counter
		wdt_reset_count();
	}
}


/**
 * will create the stack task, will store the given handles in global variables
 * @param handles task handles to test and enable
 * @param count total number of handles
 * @return NULL or on success a valid handle
 */
TaskHandle_t vCreateStackTask(TaskHandle_t *handles[], uint8_t count) {

	task_handles = handles;
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
		ulog_s("successfully created Stack Task\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		return xHandle;
	}
	return NULL;
}