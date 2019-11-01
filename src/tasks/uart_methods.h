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

// can control/setting commands
uart_command_return_t uart_command_get_serial(uint8_t cantask_id);
uart_command_return_t uart_command_get_version(uint8_t cantask_id);
uart_command_return_t uart_command_get_sw_version(uint8_t cantask_id);
uart_command_return_t uart_command_read_status(struct can_module *can_module, uint8_t cantask_id, can_flags_t *can_flags);
uart_command_return_t uart_command_set_bitrate(uint8_t cmd_len, uint8_t *cmd_buf_pntr, uint32_t *can_bitrate, can_flags_t *can_flags);
uart_command_return_t uart_command_open_can_channel(struct can_module *can_module, Can *can_instance, uint32_t *can_bitrate, can_flags_t *can_flags);
uart_command_return_t uart_command_close_can_channel(struct can_module *can_module, can_flags_t *can_flags);
uart_command_return_t uart_command_listen_only_mode(struct can_module *can_module, Can *can_instance, uint32_t *can_bitrate, can_flags_t *can_flags);

//can message commands
uart_command_return_t uart_command_send_r11bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags);
uart_command_return_t uart_command_send_11bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags);
uart_command_return_t uart_command_send_r29bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags);
uart_command_return_t uart_command_send_29bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags);

#endif //SAMC_FREERTOS_UART_COMMANDS_H
