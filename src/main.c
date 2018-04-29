#include <asf.h>
#include <stack_task.h>
#include <uart_setup.h>
#include <board_setup.h>

int main(void) {
	system_init();
	delay_init();

	/*
	 * Add Variables here
	 */
	usart_module_t debug_ulog;




	/*
	 * Add Methods here that needs to run before interrupts are enabled!
	 */
	configure_log_uart(&debug_ulog);


	/*
	 * Global Interrupts Enable!
	 */
	system_interrupt_enable_global();


	/*
	 * Add Methods here that need to run before FreeRTOS starts scheduling
	 */

	configure_ulog(&debug_ulog);
	ulog_s("prepare Tasks\r\n");
	vCreateStackTask(NULL, 0);


	ulog_s("start scheduler\r\n");
	vTaskStartScheduler();

	ulog_s("behind scheduler\r\n"); // should never be reached!!!

	do {
		// Intentionally left empty
		// program should never reach this place
	} while (true);


	return EXIT_SUCCESS;
}
