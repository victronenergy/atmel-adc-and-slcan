//
// Created by Felix Hübner on 28.04.18.
//

#ifndef SAMC_FREERTOS_BOARD_INIT_H
#define SAMC_FREERTOS_BOARD_INIT_H

/******** Typedef ********/
typedef enum {
	HW_REV_1 = 1,
	HW_REV_2 = 2, // new feature ADC got calibration resistors, i2c and leds where moved for that
} hw_rev_t;

/******** Global Variables ********/
extern hw_rev_t hw_rev;

/******** Prototypes ********/
void readSerialNumber(uint8_t serial_no[]);

#endif //SAMC_FREERTOS_BOARD_INIT_C_H
