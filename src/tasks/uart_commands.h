//
// Created by Felix Hübner on 2019-05-16.
//

#ifndef SAMC_FREERTOS_UART_COMMANDS_H
#define SAMC_FREERTOS_UART_COMMANDS_H


//TODO these values are chosen arbitrarily, set appropriate values
#define HW_VER        0x30		// hardware version
#define SW_VER        0x40		// software version
#define SW_VER_MAJOR  0x50    // software major version
#define SW_VER_MINOR  0x60    // software minor version
#define SERIAL        "2821"	// device serial number

typedef enum {
	NO_RETURN = 0,
	RETURN_ERROR = 7,
	RETURN_CR = 13,
	ERROR_BUSY = 128
} uart_command_return_t;




uart_command_return_t uart_command_get_serial(uint8_t cantask_id);
uart_command_return_t uart_command_get_version(uint8_t cantask_id);
uart_command_return_t uart_command_get_sw_version(uint8_t cantask_id);


#endif //SAMC_FREERTOS_UART_COMMANDS_H
