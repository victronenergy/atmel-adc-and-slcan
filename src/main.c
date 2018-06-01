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

static void can_send_standard_message(uint32_t id_value, uint8_t *data);

struct can_module can_instance;

static void can_send_standard_message(uint32_t id_value, uint8_t *data)
{
	uint32_t i;
	struct can_tx_element tx_element;
	can_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id_value << 18);
	for (i = 0; i < 8; i++)
	{
		tx_element.data[i] = *data;
		data++;
	}
	can_set_tx_buffer_element(&can_instance, &tx_element, CAN_TX_BUFFER_INDEX);
	can_tx_transfer_request(&can_instance, 1 << CAN_TX_BUFFER_INDEX);
}

int main(void) {
	system_init();
	delay_init();

	//TODO remove, only for debug (LED and buttons on xplained board)
	board_init();

	/*
	 * Add Variables here
	 */
	usart_module_t debug_ulog;
	usart_module_t can_uart;


/*
	// CAN TX/RX pins
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);
	pin_config.mux_position = CAN_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);
	pin_config.mux_position = CAN_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);



	// CAN Module Initialization
	struct can_config config_can;
	can_get_config_defaults(&config_can);
	can_init(&can_instance, CAN_MODULE, &config_can);
	//TODO include/replace (seems to be outdated)
	//can_switch_operation_mode(&can_instance, CAN_OPERATION_MODE_NORMAL_OPERATION);
*/



	//send CAN standard message
/*
	uint8_t data[8] = {0};
	can_send_standard_message(17,data);
*/

	/*
	 * Add Methods here that needs to run before interrupts are enabled!
	 */
	configure_log_uart(&debug_ulog);

	configure_can_uart(&can_uart);


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
	TaskHandle_t can_task = vCreateCanTask();
	task_handles[0] = &can_task;
	vCreateStackTask(&task_handles, 1);

	ulog_s("start scheduler\r\n");
	vTaskStartScheduler();

	ulog_s("behind scheduler\r\n"); // should never be reached!!!

	do {
		// Intentionally left empty
		// program should never reach this place
	} while (true);


	return EXIT_SUCCESS;
}
