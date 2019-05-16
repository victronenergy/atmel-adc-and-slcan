//
// Created by Felix Hübner on 2019-05-16.
//

#include "conf_can.h"
#include "can.h"
#include "log.h"
#include "usb.h"
#include "slcan.h"
#include "uart_commands.h"


uart_command_return_t uart_command_get_serial(uint8_t cantask_id) {
	if (!usb_putc(GET_SERIAL,cantask_id))
		return ERROR_BUSY;
	if (!usb_puts((uint8_t *) SERIAL,cantask_id))
		return ERROR_BUSY;
	return RETURN_CR;
}


uart_command_return_t uart_command_get_version(uint8_t cantask_id) {
	if (!usb_putc(GET_VERSION,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(HW_VER,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(SW_VER,cantask_id))
		return ERROR_BUSY;
	return RETURN_CR;
}


uart_command_return_t uart_command_get_sw_version(uint8_t cantask_id) {
	if (!usb_putc(GET_SW_VERSION,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(SW_VER_MAJOR,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(SW_VER_MINOR,cantask_id))
		return ERROR_BUSY;
	return RETURN_CR;
}
