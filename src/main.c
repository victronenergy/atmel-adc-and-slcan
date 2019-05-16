#include <asf.h>
#include <stack_task.h>
#include <comm_interface_setup.h>
#include <board_setup.h>
#include <samc21_slcan_adc.h>
#include "stdint.h"
#include "conf_can.h"
#include "can_task.h"
#include "can.h"


int main(void) {
	system_init();
	delay_init();

	//TODO remove, only for debug (LED and buttons on xplained board)
	board_init();

	/*
	 * Add Variables here
	 */
	usart_module_t debug_ulog;
	usart_module_t usbcan0_instance;


	/*
	 * Add Methods here that need to run before interrupts are enabled!
	 */
	configure_log_uart(&debug_ulog);

	configure_uart_can0(&usbcan0_instance);

	cantask_params params_task0;
	params_task0.task_id = CANTASK_ID_0;
	params_task0.usart_instance = &usbcan0_instance;
	params_task0.can_instance = CAN0_MODULE;

	/*
	 * Global Interrupts Enable!
	 */
	system_interrupt_enable_global();


	/*
	 * Add Methods here that need to run before FreeRTOS starts scheduling
	 */

	configure_ulog(&debug_ulog);
	ulog_s("prepare Tasks\r\n");
	TaskHandle_t task_handles[2];
	TaskHandle_t can_task0 = vCreateCanTask(&params_task0);
	task_handles[0] = &can_task0;
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
