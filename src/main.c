#include <asf.h>
#include <stack_task.h>
#include <comm_interface_setup.h>
#include <board_setup.h>
#include <samc21_slcan_adc.h>
#include "adc_task.h"
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
	usart_module_t uartcan_instance;
	adc_module_t adc_instance0;
	adc_module_t adc_instance1;
	dma_resource_t dma_resource0;
	dma_resource_t dma_resource1;

	/*
	 * Add Methods here that need to run before interrupts are enabled!
	 */
	configure_log_uart(&debug_ulog);

	configure_uart_can0(&uartcan_instance);

	cantask_params params_task;
	params_task.task_id = CANTASK_ID_0;
	params_task.usart_instance = &uartcan_instance;
	params_task.can_instance = CAN0_MODULE;

	adctask_params adc_params;
	adc_params.adc_instance0 = &adc_instance0;
	adc_params.adc_instance1 = &adc_instance1;
	adc_params.dma_resource0 = &dma_resource0;
	adc_params.dma_resource1 = &dma_resource1;

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
	TaskHandle_t can_task = vCreateCanTask(&params_task);
	TaskHandle_t adc_task = vCreateAdcTask(&adc_params);
	task_handles[0] = &can_task;
	task_handles[1] = &adc_task;
	vCreateStackTask((TaskHandle_t **) &task_handles, 2);

	ulog_s("start scheduler\r\n");
	vTaskStartScheduler();

	ulog_s("behind scheduler\r\n"); // should never be reached!!!

	do {
		// Intentionally left empty
		// program should never reach this place
	} while (true);


	return EXIT_SUCCESS;
}
