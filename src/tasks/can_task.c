//
// Created by stekreis on 25.05.18.
//

#include <log.h>
#include "can_task.h"
#include <usb.h>
#include <cmsis/samc21/include/component/can.h>
#include <can.h>
#include "uart_methods.h"
#include "can_methods.h"


/******** Internal Prototypes ********/
void vCanTask(void *pvParameters);


/******** Methods ********/
/**
 * CanTask handles data incoming from the CAN interface(s) as well as via the UART interface and transfer it to the
 * other interface
 *
 *
 * @param pvParameters contains the pointers to the modules and structs that should not live inside the task, but as
 * local variable in main.
 */
void vCanTask(void *pvParameters) {

	vTaskDelay((const TickType_t) 10);

	// CAN status flags
	can_flags_t can_flags = {.bus_on = false, .init_complete = false, .tx_busy = false};

	// task parameters (uart+can instances, task id)
	cantask_params *params = (cantask_params *) pvParameters;
	Can *can_instance = params->can_instance;
	usart_module_t *usart_instance = params->usart_instance;
	uint8_t cantask_id = params->task_id;

	// can structure to control the can hw instance
	struct can_module can_module;
	uint32_t can_bitrate = 0;
	uint8_t sequence_counter = 0;
	uint32_t buf_num = 0;
	uint8_t *buf = NULL;

	// uart data struct for rx/tx buffers (allows multi buffering)
	usart_buf_t usart_buf;


	// initially both LEDs off
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
	port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);

	configure_usart_callbacks(usart_instance, cantask_id);

	start_canbus_usart(usart_instance, &usart_buf, cantask_id);

	ulog_s("start CAN Task ");
	uint8_t tmp = (uint8_t) (cantask_id + 0x30);
	ulog( &tmp,1);
	ulog_s("...\r\n");



	for (;;) {
		/**
		 * UART TO CAN HANDLING
		 */
		enum_usb_return_t new_cmd_available = get_complete_cmd(&buf, &buf_num, cantask_id);
		if (new_cmd_available == NEW_CMD || new_cmd_available == BUF_OVERRUN) {
			// Execute USB command and return status to terminal
			uint8_t r = exec_uart_cmd(&can_module, can_instance, buf, &can_flags, cantask_id, &can_bitrate);
			if (r != NO_RETURN && r != ERROR_BUSY) { // check if we have to send something back to the host.
				usb_putc(r, cantask_id);
				usb_send(usart_instance, cantask_id);
			}
			// flush command buffer
			if (r != ERROR_BUSY) {
				port_pin_set_output_level(PIN_PA19, false);
				port_pin_set_output_level(PIN_PA19, true);
				clear_cmd_buf(usart_instance, cantask_id, buf_num);
			}
		}

		/**
		 * CAN TO UART HANDLING
		 */
		if (can_flags.bus_on) {
			bool new_can_message = check_and_transfer_can_message_to_uart(&can_module, cantask_id, &sequence_counter);
			if (new_can_message) {
				usb_send(usart_instance, cantask_id);
			}
		}
		flush_clog();

		// zero task delay used for rescheduling, to prevent interference between
		// simultaneous uart sending of both tasks
		vTaskDelay(0);
	}
}


/**
 * will be called from main to set up the can task, will suspend the task after creation
 *
 *
 * @param params contains the pointers to the modules and structs that should not life inside the task, but as
 * local variable in main.
 * @return NULL or on success a valid handle
 */
TaskHandle_t vCreateCanTask(cantask_params *params) {

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	char task_name[8] = "cTASK_";
	task_name[6] = (uint8_t) (params->task_id+0x30);
	task_name[7] = '\0';

	ulog_s("creating CAN Task ");
	ulog((uint8_t*) &task_name[6],1);
	ulog_s("...\r\n");

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vCanTask,       /* Function that implements the task. */
			task_name,          /* Text name for the task. */
			350,      /* Stack size in words, not bytes. */
			params,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			&xHandle);      /* Used to pass out the created task's handle. */

	if (xReturned == pdPASS) {
		ulog_s("successfully created CAN Task ");
		ulog((uint8_t*) &task_name[6],1);
		ulog_s("\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;


}

