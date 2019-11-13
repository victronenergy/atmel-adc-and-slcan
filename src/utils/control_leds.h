//
// Created by Felix Hübner on 2019-11-13.
//

#ifndef SAMC_FREERTOS_CONTROL_LEDS_H
#define SAMC_FREERTOS_CONTROL_LEDS_H

typedef enum {
	GREEN_LED,
	RED_LED,
} led_t;

/**
 * setup the led pins
 */
void setup_leds(void);

/**
 * set a specific led to given state
 * @param id ID of the given led
 * @param state new state
 */
void set_led(led_t id, bool state);


#endif //SAMC_FREERTOS_CONTROL_LEDS_H
