//
// Created by Felix Hübner on 2019-05-16.
//

#ifndef SAMC_FREERTOS_UART_COMMANDS_H
#define SAMC_FREERTOS_UART_COMMANDS_H

#include "can_methods.h"

/******** Defines ********/
//TODO these values are chosen arbitrarily, set appropriate values
#define HW_VER        0x30		// hardware version
#define SW_VER        0x40		// software version
#define SW_VER_MAJOR  0x50    // software major version
#define SW_VER_MINOR  0x60    // software minor version
#define SERIAL        "2821"	// device serial number


/******** Typedef ********/
typedef enum {
	NO_RETURN = 0,
	RETURN_ERROR = 7,
	RETURN_CR = 13,
	ERROR_BUSY = 128
} uart_command_return_t;

/******** Prototypes ********/
/**
 * will be called from CAN-TASK and processes the command given in cmd_buf from uart receive
 *
 * @param can_module can-module to work with
 * @param can_instance can-instance to work with
 * @param cmd_buf cmd_buffer with the command that has to be processed
 * @param can_flags struct with can_flags
 * @param cantask_id task number
 * @param can_bitrate pointer to a field to store the selected bitrate in, the bitrate is set before the can-module is initialized
 * @return return the result of the proccessing, possible options are: NO_RETURN; RETURN_CR; RETURN_ERROR; ERROR_BUSY
 */
uart_command_return_t exec_uart_cmd(struct can_module *can_module, Can *can_instance, uint8_t *cmd_buf, can_flags_t *can_flags, uint8_t cantask_id, uint32_t *can_bitrate);

#endif //SAMC_FREERTOS_UART_COMMANDS_H
