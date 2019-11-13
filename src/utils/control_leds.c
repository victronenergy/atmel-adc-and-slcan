//
// Created by Felix Hübner on 2019-11-13.
//

#include <asf.h>
#include "control_leds.h"
#include "board_setup.h"


/**
 * setup the led pins
 */
void setup_leds(void) {
	struct port_config pin_conf_led;
	port_get_config_defaults(&pin_conf_led);

	pin_conf_led.direction  = PORT_PIN_DIR_OUTPUT;
	switch (hw_rev) {
		case HW_REV_1:
			port_pin_set_config(HW_REV_1_LEDPIN_C21_GREEN, &pin_conf_led);
			port_pin_set_config(HW_REV_1_LEDPIN_C21_RED, &pin_conf_led);
			break;
		case HW_REV_2:
			port_pin_set_config(HW_REV_2_LEDPIN_C21_GREEN, &pin_conf_led);
			port_pin_set_config(HW_REV_2_LEDPIN_C21_RED, &pin_conf_led);
			break;
		default:
			break;
	}
}

/**
 * set a specific led to given state
 * @param id ID of the given led
 * @param state new state
 */
void set_led(led_t id, bool state) {
	uint8_t led = 0;
	switch(hw_rev) {
		case HW_REV_1:
			if (id == GREEN_LED)
				led = HW_REV_1_LEDPIN_C21_GREEN;
			else
				led = HW_REV_1_LEDPIN_C21_RED;
			break;
		case HW_REV_2:
			if (id == GREEN_LED)
				led = HW_REV_2_LEDPIN_C21_GREEN;
			else
				led = HW_REV_2_LEDPIN_C21_RED;
			break;
		default:
			break;
	}
	port_pin_set_output_level(led, state);
}