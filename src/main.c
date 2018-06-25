#include <asf.h>
#include <stack_task.h>
#include <uart_setup.h>
#include <board_setup.h>
#include "stdint.h"
#include "conf_can.h"
#include "can_task.h"
#include "can.h"


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
	usart_module_t usbcan0_instance;
	usart_module_t usbcan1_instance;

	struct can_module can0_instance;
	struct can_module can1_instance;


	/*
	 * Add Methods here that need to run before interrupts are enabled!
	 */
	configure_log_uart(&debug_ulog);

	configure_can0(&can0_instance);

	configure_usbcan0(&usbcan0_instance);

	configure_usbcan1(&usbcan1_instance);

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
	TaskHandle_t can_task = vCreateCanTask(&usbcan0_instance, &can0_instance);
	task_handles[0] = &can_task;
	vCreateStackTask((TaskHandle_t **) &task_handles, 1);

/*	while(1){

		delay_ms(1000);

		port_pin_toggle_output_level(PIN_PA13);
		port_pin_toggle_output_level(PIN_PA12);


		ulog_s("yalla\r\n");
		usart_write_buffer_wait(&usbcan0_instance, (const uint8_t *)"\r\nTEST0:", 8);
		usart_write_buffer_wait(&usbcan1_instance, (const uint8_t *)"\r\nTEST1:", 8);
	}
*/
	ulog_s("start scheduler\r\n");
	vTaskStartScheduler();

	ulog_s("behind scheduler\r\n"); // should never be reached!!!

	do {
		// Intentionally left empty
		// program should never reach this place
	} while (true);


	return EXIT_SUCCESS;
}
