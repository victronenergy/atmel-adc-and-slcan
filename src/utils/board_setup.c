#include "adc_task.h"
#include "control_leds.h"
#include <asf.h>
#include <board_setup.h>
#include <log.h>

/******** Internal Prototypes ********/
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed portCHAR *pcTaskName);
uint8_t readHWrev(void);

/******** Global Variables ********/
hw_rev_t hw_rev = HW_REV_1;

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

	// fill global variable
	hw_rev = readHWrev();

	// setup LEDs
	setup_leds();

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
	hw_rev_t rev = 0;
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


/**
 * read the HW Serialnumber and write it to a given array
 * @param serial_no array to store the HW Serial number in, has to be at least 16 bytes long
 */
void readSerialNumber(uint8_t serial_no[]) {
	// copy serialnumber to ram!
	// Word 0
	uint32_t tmp =  *((uint32_t *) 0x0080A00C);
	serial_no[0] = (uint8_t) (tmp >> 24);
	serial_no[1] = (uint8_t) (tmp >> 16);
	serial_no[2] = (uint8_t) (tmp >> 8);
	serial_no[3] = (uint8_t) (tmp >> 0);

	// Word 1-3
	for (uint8_t i = 0; i<9;i+=4) {
		tmp = *((uint32_t *) (0x0080A040+i));
		serial_no[4+(i)] = (uint8_t) (tmp >> 24);
		serial_no[5+(i)] = (uint8_t) (tmp >> 16);
		serial_no[6+(i)] = (uint8_t) (tmp >> 8);
		serial_no[7+(i)] = (uint8_t) (tmp >> 0);
	}
}