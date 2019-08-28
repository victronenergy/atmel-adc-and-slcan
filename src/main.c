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
#include <component/rstc.h>


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
	i2c_module_t i2c_slave_instance;

	/*
	 * make sure the software device instance structs are 0x00 on all fields!
	 */
	memset(&debug_ulog, 0x00, sizeof(usart_module_t));
	memset(&uartcan_instance, 0x00, sizeof(usart_module_t));
	memset(&adc_instance0, 0x00, sizeof(adc_module_t));
	memset(&adc_instance1, 0x00, sizeof(adc_module_t));
	memset(&dma_resource0, 0x00, sizeof(dma_resource_t));
	memset(&dma_resource1, 0x00, sizeof(dma_resource_t));
	memset(&i2c_slave_instance, 0x00, sizeof(i2c_module_t));

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
	adc_params.i2c_instance = &i2c_slave_instance;

	/*
	 * Global Interrupts Enable!
	 */
	system_interrupt_enable_global();


	/*
	 * Add Methods here that need to run before FreeRTOS starts scheduling
	 */

	configure_ulog(&debug_ulog);
	ulog_s("\r\n");

	/*
	 * check fuses, write if not 'correct'
	 */
	{
		struct nvm_fusebits fuses;
		nvm_get_fuses(&fuses);
		if(fuses.bodvdd_enable != true) {
			ulog_s("update fuses");
			fuses.bodvdd_enable = true;
			fuses.bodvdd_action = NVM_BOD33_ACTION_RESET;
			fuses.bodvdd_hysteresis = true;
			fuses.bodvdd_level = 8;
			if (nvm_set_fuses(&fuses) != STATUS_OK) {
				ulog_s(" failed\r\n");
			} else {
				ulog_s(" ok\r\n");
			}
		}
	}
	/**
	0  Power On Reset
	1  Brown Out CORE Detector Reset
	2  Brown Out VDD Detector Reset
	3  Reserved
	4  External Reset
	5  Watchdog Reset
	6  System Reset Request
	7  Reserved
	 */
	ulog_s("Reset info: 0x");
	enum system_reset_cause cause = system_get_reset_cause();
	xlog((uint8_t *) &cause, 1);
	ulog_s(" - ");
	switch(cause) {
		case SYSTEM_RESET_CAUSE_POR:
			ulog_s("Power On Reset");
			break;
		case SYSTEM_RESET_CAUSE_BODCORE:
			ulog_s("Brown Out CORE Detector Reset");
			break;
		case SYSTEM_RESET_CAUSE_BODVDD:
			ulog_s("Brown Out VDD Detector Reset");
			break;
		case SYSTEM_RESET_CAUSE_EXTERNAL_RESET:
			ulog_s("External Reset");
			break;
		case SYSTEM_RESET_CAUSE_WDT:
			ulog_s("Watchdog Reset");
			break;
		case SYSTEM_RESET_CAUSE_SOFTWARE:
			ulog_s("System Reset Request");
			break;
		default:
			ulog_s("Other Reset");
			break;
	}
	ulog_s("\r\n");

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
