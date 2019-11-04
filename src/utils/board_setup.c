#include <asf.h>
#include <board_setup.h>
#include <log.h>

/******** Internal Prototypes ********/
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed portCHAR *pcTaskName);


/******** Methods ********/
/**
 * will be called from RTOS if the memory region of a task overwritten by local variables
 * @param pxTask handle to the task
 * @param pcTaskName name of the task
 */
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed portCHAR *pcTaskName) {
	system_interrupt_enable_global();
	ulog_s("\r\nFATAL: task (");
	ulog_s((const char*) pcTaskName);
	ulog_s(") overflowed the stack! i will stop here!\r\n");
	for (;;) {
		ulog_s("\r\nINFO: waiting in StackOverflowHook");
		delay_s(2);
	};
}


/**
 * setup of pins and modules
 */
void board_init(void) {
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	// LED
	struct port_config pin_conf_led;
	port_get_config_defaults(&pin_conf_led);

	pin_conf_led.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA12, &pin_conf_led);
	port_pin_set_config(PIN_PA13, &pin_conf_led);

	struct port_config debug_port;
	port_get_config_defaults(&debug_port);

	debug_port.direction  = PORT_PIN_DIR_OUTPUT;


	//debug pins
	port_pin_set_config(PIN_PA15, &debug_port);
	port_pin_set_config(PIN_PA14, &debug_port);
	port_pin_set_config(PIN_PA19, &debug_port);
	port_pin_set_config(PIN_PA20, &debug_port);

	port_pin_set_output_level(PIN_PA15, false);
	port_pin_set_output_level(PIN_PA14, false);
	port_pin_set_output_level(PIN_PA19, false);
	port_pin_set_output_level(PIN_PA20, false);


	/* Set up the CAN TX/RX pins */
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);
	pin_config.mux_position = CAN0_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN0_TX_PIN, &pin_config);
	pin_config.mux_position = CAN0_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN0_RX_PIN, &pin_config);

}


/**
 * read HW-REV from input pins, there are 3 pins that have resistor places to read that version from
 * @return return the number read from the resistors
 */
uint8_t readHWrev(void) {
	uint8_t rev = 0;
    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);

    /* Set buttons as inputs */

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
    pin_conf.input_pull = PORT_PIN_PULL_DOWN;
    pin_conf.powersave = false;
    //enable input and pulldown on hw rev pins
    port_pin_set_config(HW_REV_DETECTION_0, &pin_conf);
    port_pin_set_config(HW_REV_DETECTION_1, &pin_conf);
    port_pin_set_config(HW_REV_DETECTION_2, &pin_conf);

    //read status of pins
    rev |= port_pin_get_input_level(HW_REV_DETECTION_0);
    rev |= port_pin_get_input_level(HW_REV_DETECTION_1) << 1;
    rev |= port_pin_get_input_level(HW_REV_DETECTION_2) << 2;

    //disable hw rev pins
    pin_conf.powersave = true;
    port_pin_set_config(HW_REV_DETECTION_0, &pin_conf);
    port_pin_set_config(HW_REV_DETECTION_1, &pin_conf);
    port_pin_set_config(HW_REV_DETECTION_2, &pin_conf);

    return rev;
}