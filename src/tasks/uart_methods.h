//
// Created by Felix Hübner on 2019-05-16.
//

#ifndef SAMC_FREERTOS_UART_COMMANDS_H
#define SAMC_FREERTOS_UART_COMMANDS_H

#include "can_methods.h"

/******** Defines ********/
#ifndef SW_VERSION
	#define SW_VERSION 		((uint8_t) -1)
#endif

#define SW_VER        ((uint8_t) SW_VERSION)	// software version
#define SW_VER_MAJOR  0x20						// software major version
#define SW_VER_MINOR  SW_VER					// software minor version


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

/**
 * can be called to open the physical can_bus interface
 * @param can_module can-module to work with
 * @param can_instance can-instance to work with
 * @param can_bitrate pointer to a field to store the selected bitrate in, the bitrate is set before the can-module is initialized
 * @param can_flags struct with can_flags
 * @return
 */
uart_command_return_t uart_command_open_can_channel(struct can_module *can_module, Can *can_instance, uint32_t *can_bitrate, can_flags_t *can_flags);

#endif //SAMC_FREERTOS_UART_COMMANDS_H
