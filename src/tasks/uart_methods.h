//
// Created by Felix Hübner on 2019-05-16.
//

#ifndef SAMC_FREERTOS_UART_COMMANDS_H
#define SAMC_FREERTOS_UART_COMMANDS_H

#include "can_methods.h"

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


uart_command_return_t exec_uart_cmd(struct can_module *can_module, Can *can_instance, uint8_t *cmd_buf, can_flags_t *can_flags, uint8_t cantask_id, uint32_t *can_bitrate);

#endif //SAMC_FREERTOS_UART_COMMANDS_H
