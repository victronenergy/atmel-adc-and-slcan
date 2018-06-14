#include <asf.h>
#include <stack_task.h>
#include <uart_setup.h>
#include <board_setup.h>
#include "stdint.h"
#include "conf_can.h"
#include "can_task.h"
#include "can.h"
#include "samc21_xplained_pro.h"


//TODO set correct value
#define CAN_TX_BUFFER_INDEX 0


int main(void) {
	system_init();
	delay_init();

	//TODO remove, only for debug (LED and buttons on xplained board)
	board_init();

	/*
	 * Add Variables here
	 */
	usart_module_t debug_ulog;
	usart_module_t usart_instance;

	struct can_module can_instance;


	/*
	 * Add Methods here that need to run before interrupts are enabled!
	 */
	configure_log_uart(&debug_ulog);

	//TODO this is resposible for error at start
	configure_can(&can_instance);

	configure_uart(&usart_instance);

	/*
	 * Global Interrupts Enable!
	 */
	system_interrupt_enable_global();


	/*
	 * Add Methods here that need to run before FreeRTOS starts scheduling
	 */

	configure_ulog(&debug_ulog);
	ulog_s("prepare Tasks\r\n");
	TaskHandle_t task_handles[1];
	TaskHandle_t can_task = vCreateCanTask(&usart_instance, &can_instance);
	task_handles[0] = &can_task;
	vCreateStackTask((TaskHandle_t **) &task_handles, 1);

	ulog_s("start scheduler\r\n");
	vTaskStartScheduler();

	ulog_s("behind scheduler\r\n"); // should never be reached!!!

	do {
		// Intentionally left empty
		// program should never reach this place
	} while (true);


	return EXIT_SUCCESS;
}
