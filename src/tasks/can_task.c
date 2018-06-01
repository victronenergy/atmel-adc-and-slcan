//
// Created by stekreis on 25.05.18.
//

#include <log.h>
#include <samc21_xplained_pro.h>
#include "can_task.h"


void vCanTask(void *pvParameters);

void vCanTask(void *pvParameters) {
	ulog_s("cantask begin loop\r\n");

	vTaskDelay((const TickType_t) 100);

	//TODO prepare CAN interface

	for (;;) {
		vTaskDelay((const TickType_t) 100);
		port_pin_toggle_output_level(LED_0_PIN);

		// TODO perform CAN task

	}
}

TaskHandle_t vCreateCanTask(){

	ulog_s("creating CAN Task...\r\n");

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vCanTask,       /* Function that implements the task. */
			"canTASK",          /* Text name for the task. */
			250,      /* Stack size in words, not bytes. */
			NULL,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY + 5,/* Priority at which the task is created. */
			&xHandle);      /* Used to pass out the created task's handle. */

	if( xReturned == pdPASS )
	{
		ulog_s("\r\nsuccessfully created CAN Task\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		//vTaskDelete( xHandle );
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;



}