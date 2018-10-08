//*****************************************************************************
//
// Title        : AVR based USB<>CAN adaptor
// Authors      : Michael Wolf
// File Name    : 'usb.c'
// Date         : August 24, 2005
// Version      : 1.00
// Target MCU   : Atmel AVR ATmega162
// Editor Tabs  : 2
//
// NOTE: The authors in no way will be responsible for damages that you
//       coul'd be using this code.
//       Use this code at your own risk.
//
//       This code is distributed under the GNU Public License
//       which can be found at http://www.gnu.org/licenses/gpl.txt
//
// Change Log
//
// Version  When        Who           What
// -------  ----        ---           ----
// 1.00     24/08/2005  Michael Wolf  Initial Release
//
//*****************************************************************************
#include <stdint.h>
#include <samc21_usbcan.h>
#include <can_task.h>

#include "usb.h"

/*
**---------------------------------------------------------------------------
**
** Abstract: USB get char
**
**
** Parameters: none
**
**
** Returns: 0 = no char received
**          else ASCII char
**
**
**---------------------------------------------------------------------------
*/

void usart_read_callback_cantask0(struct usart_module *const usart_module);
void usart_write_callback_cantask0(struct usart_module *const usart_module);
void usart_read_callback_cantask1(struct usart_module *const usart_module);
void usart_write_callback_cantask1(struct usart_module *const usart_module);


usart_buf_t *usart_buf_can0 = NULL;
usart_buf_t *usart_buf_can1 = NULL;

void usart_read_callback_cantask0(struct usart_module *const usart_module) {
//	port_pin_set_output_level(PIN_PA10, true);
	if (usart_buf_can0 == NULL) {
		return;
	}
//	port_pin_set_output_level(PIN_PA10, false);
//	port_pin_set_output_level(PIN_PA10, true);
	if((usart_buf_can0->usart_buf_rx[usart_buf_can0->fill_rx][usart_buf_can0->rx_insert_pos]) == CR){
		taskENTER_CRITICAL();
		usart_buf_can0->usart_buf_rx[usart_buf_can0->fill_rx][usart_buf_can0->rx_insert_pos] = 0; // remove CR from receive buffer!
		usart_buf_can0->rx_pending_map |=  (1 << usart_buf_can0->fill_rx);
		if (usart_buf_can0->rx_pending_map & ((usart_buf_can0->fill_rx + 1) % USB_CMD_BUF_COUNT)) {
			//if this happens the commands are not processed fast enough and we are now starting to overwrite the last stored command!
			usart_buf_can0->buf_overrun = true;
		}
		usart_buf_can0->fill_rx = (uint8_t) ((usart_buf_can0->fill_rx + 1) % USB_CMD_BUF_COUNT);
		usart_buf_can0->rx_insert_pos = -1;
		taskEXIT_CRITICAL();
	}
//	port_pin_set_output_level(PIN_PA10, false);
//	port_pin_set_output_level(PIN_PA10, true);
	usart_buf_can0->rx_insert_pos = (uint8_t) ((usart_buf_can0->rx_insert_pos + 1) % USB_CMD_BUF_SIZE);
//	port_pin_set_output_level(PIN_PA10, false);
//	port_pin_set_output_level(PIN_PA10, true);
	usart_read_job(usart_module, (uint16_t *) &(usart_buf_can0->usart_buf_rx[usart_buf_can0->fill_rx][usart_buf_can0->rx_insert_pos]));
//	port_pin_set_output_level(PIN_PA10, false);
}

void usart_read_callback_cantask1(struct usart_module *const usart_module) {
	if (usart_buf_can1 == NULL) {
		return;
	}
	if((usart_buf_can1->usart_buf_rx[usart_buf_can1->fill_rx][usart_buf_can1->rx_insert_pos]) == CR){
		taskENTER_CRITICAL();
		usart_buf_can1->usart_buf_rx[usart_buf_can1->fill_rx][usart_buf_can1->rx_insert_pos] = 0; // remove CR from receive buffer!
		usart_buf_can1->rx_pending_map |=  (1 << usart_buf_can1->fill_rx);
		if (usart_buf_can1->rx_pending_map & ((usart_buf_can1->fill_rx + 1) % USB_CMD_BUF_COUNT)) {
			//if this happens the commands are not processed fast enough and we are now starting to overwrite the last stored command!
			usart_buf_can1->buf_overrun = true;
		}
		usart_buf_can1->fill_rx = (uint8_t) ((usart_buf_can1->fill_rx + 1) % USB_CMD_BUF_COUNT);
		usart_buf_can1->rx_insert_pos = -1;
		taskEXIT_CRITICAL();
	}
	usart_buf_can1->rx_insert_pos = (uint8_t) ((usart_buf_can1->rx_insert_pos + 1) % USB_CMD_BUF_SIZE);
	usart_read_job(usart_module, (uint16_t *) &(usart_buf_can1->usart_buf_rx[usart_buf_can1->fill_rx][usart_buf_can1->rx_insert_pos]));
}

void usart_write_callback_cantask0(struct usart_module *const usart_module) {
	if (usart_buf_can0 == NULL) {
		return;
	}

	if(usart_buf_can0->tx_pending_map){
//		ulog_s(" map: ");
//		xlog( (uint8_t *) &usart_buf_can0->tx_pending_map, 1);
//		ulog_s(" read: ");
//		xlog( (uint8_t *) &usart_buf_can0->read_tx, 1);
		taskENTER_CRITICAL();
		enum status_code result = usart_write_buffer_job(usart_module, &(usart_buf_can0->usart_buf_tx[usart_buf_can0->read_tx][0]), (uint16_t) (usart_buf_can0->buf_tx_len[usart_buf_can0->read_tx]));
		if (result == STATUS_OK) {
			//successfull started job
			usart_buf_can0->buf_tx_len[usart_buf_can0->read_tx] = 0;
			usart_buf_can0->tx_pending_map &= ~(1 << usart_buf_can0->read_tx);
			usart_buf_can0->read_tx = (uint8_t) ((usart_buf_can0->read_tx + 1) % USB_CMD_BUF_COUNT);
		}
		taskEXIT_CRITICAL();
	}
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
}

void usart_write_callback_cantask1(struct usart_module *const usart_module) {
	if (usart_buf_can1 == NULL) {
		return;
	}

	if(usart_buf_can1->tx_pending_map){
//		ulog_s(" map: ");
//		xlog( (uint8_t *) &usart_buf_can1->tx_pending_map, 1);
//		ulog_s(" read: ");
//		xlog( (uint8_t *) &usart_buf_can1->read_tx, 1);
		taskENTER_CRITICAL();
		enum status_code result = usart_write_buffer_job(usart_module, &(usart_buf_can1->usart_buf_tx[usart_buf_can1->read_tx][0]), (uint16_t) (usart_buf_can1->buf_tx_len[usart_buf_can1->read_tx]));
		if (result == STATUS_OK) {
			//successfull started job
			usart_buf_can1->buf_tx_len[usart_buf_can1->read_tx] = 0;
			usart_buf_can1->tx_pending_map &= ~(1 << usart_buf_can1->read_tx);
			usart_buf_can1->read_tx = (uint8_t) ((usart_buf_can1->read_tx + 1) % USB_CMD_BUF_COUNT);
		}
		taskEXIT_CRITICAL();
	}
	//TODO adapt to other LED than callback0 when available
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
}

void configure_usart_callbacks(usart_module_t *usart_instance, uint8_t cantask_id) {
	if(cantask_id == CANTASK_ID_0) {
		usart_register_callback(usart_instance,	usart_write_callback_cantask0, USART_CALLBACK_BUFFER_TRANSMITTED);
		usart_register_callback(usart_instance,	usart_read_callback_cantask0, USART_CALLBACK_BUFFER_RECEIVED);
	}else{
		usart_register_callback(usart_instance,	usart_write_callback_cantask1, USART_CALLBACK_BUFFER_TRANSMITTED);
		usart_register_callback(usart_instance, usart_read_callback_cantask1, USART_CALLBACK_BUFFER_RECEIVED);
	}
	// valid for both calls as it depends on the usart instance
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


// extract cmd from buffer
enum_usb_return_t get_complete_cmd(uint8_t **cmd_buf, uint32_t *buf_num, uint8_t cantask_id) {
	usart_buf_t *buf;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL) {
		return USB_ERROR;
	}
//	ulog_s(" map: ");
//	xlog((uint8_t *) &buf->rx_pending_map, 1);
//	ulog_s(" fill: ");
//	xlog(&buf->fill_rx, 1);
//	ulog_s(" read: ");
//	xlog(&buf->read_rx, 1);
	if (buf->rx_pending_map & (1 << buf->read_rx)) {
		uint8_t pos = buf->read_rx;
		buf->read_rx = (uint8_t) ((buf->read_rx + 1) % USB_CMD_BUF_COUNT);
		*cmd_buf = buf->usart_buf_rx[pos];
		*buf_num = pos;

/*		ulog_s("\r\nBUF0: ");
		xlog(buf->usart_buf_rx[0], CMD_BUFFER_LENGTH);
		ulog_s("\r\nBUF1: ");
		xlog(buf->usart_buf_rx[1], CMD_BUFFER_LENGTH);
		c_log('n');*/

		if (buf->buf_overrun) {
			//TODO test overflow
//			ulog_s(" ovr ");
			return BUF_OVERRUN;
		}
		return NEW_CMD;

	}
	return NO_CMD;
}

bool clear_cmd_buf(uint8_t cantask_id, uint32_t buf_num) {
	usart_buf_t *buf;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL) {
		return false;
	}
//	ulog_s(" clear: ");
//	xlog((uint8_t *) &buf_num,1);
	if (buf_num < USB_CMD_BUF_COUNT) {
		buf->rx_pending_map &= ~(1 << buf_num);
//		buf->rx_pending_map = 0;
//		for (uint32_t i = 0; i < USB_CMD_BUF_COUNT; i++) {
//			memset(buf->usart_buf_rx[i], 0x00, USB_CMD_BUF_SIZE);
//		}
		memset(buf->usart_buf_rx[buf_num], 0x00, USB_CMD_BUF_SIZE);
		return true;
	}
	return false;
}

bool start_canbus_usart(usart_module_t *usart_instance, usart_buf_t *buf_struct, uint8_t cantask_id){
	if (buf_struct == NULL) {
		return false;
	}
	if(cantask_id == CANTASK_ID_0) {
		usart_buf_can0 = buf_struct;
	} else {
		usart_buf_can1 = buf_struct;
	}

	//reset struct to default values
	buf_struct->rx_pending_map = 0;
	buf_struct->tx_pending_map = 0;
	buf_struct->buf_overrun = false;
	buf_struct->rx_insert_pos = 0;
	buf_struct->tx_insert_pos = 0;
	buf_struct->fill_rx = 0;
	buf_struct->fill_tx = 0;
	buf_struct->read_rx = 0;
	buf_struct->read_tx = 0;
	for (uint32_t i = 0; i < USB_CMD_BUF_COUNT; i++) {
		memset(buf_struct->usart_buf_rx[i], 0x00, USB_CMD_BUF_SIZE);
		memset(buf_struct->usart_buf_tx[i], 0x00, USB_CMD_BUF_SIZE);
		buf_struct->buf_tx_len[i] = 0;
	}

	return usart_read_job(usart_instance, (uint16_t *) &(buf_struct->usart_buf_rx[buf_struct->fill_rx][buf_struct->rx_insert_pos]));
}


/*
**---------------------------------------------------------------------------
**
** Abstract: USB put char
**
**
** Parameters: character to send via USB
**
**
** Returns: none
**
**
**---------------------------------------------------------------------------
*/

enum status_code usb_send(usart_module_t *module, uint8_t cantask_id) {
	// check which task wants to send data
	usart_buf_t *buf;
	usart_callback_t callback_func = NULL;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
		callback_func = usart_write_callback_cantask0;
	} else {
		buf = usart_buf_can1;
		callback_func = usart_write_callback_cantask1;
	}
	if (buf == NULL) {
		return STATUS_ERR_BAD_DATA;
	}

	enum status_code result = STATUS_OK;
	taskENTER_CRITICAL();
	if (buf->tx_insert_pos > 0){
		buf->buf_tx_len[buf->fill_tx] = buf->tx_insert_pos;
		if (buf->tx_pending_map & ((buf->fill_tx + 1) % USB_CMD_BUF_COUNT)) {
			//if this happens the commands are not processed fast enough and we are now starting to overwrite the last stored command!
			result = STATUS_ERR_OVERFLOW;
		}
		buf->tx_pending_map |=  (1 << buf->fill_tx);
		buf->fill_tx = (uint8_t) ((buf->fill_tx + 1) % USB_CMD_BUF_COUNT);
		buf->tx_insert_pos = 0;
	}
	taskEXIT_CRITICAL();

	//start sending data if available
	callback_func(module);
	return result;
}


/*
 * appends a char to the sequence in the buffer. does not send any data.
 *
 * @param usart_instance
 * @param tx_byte
 * @param cantask_id
 */
bool usb_putc(uint8_t tx_byte, uint8_t cantask_id) {
	// check which task wants to send data
	usart_buf_t *buf;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL) {
		return false;
	}
	buf->usart_buf_tx[buf->fill_tx][buf->tx_insert_pos] = tx_byte;
	buf->tx_insert_pos = (uint8_t) ((buf->tx_insert_pos + 1) % USB_CMD_BUF_SIZE);
	return true;
}




/*
**---------------------------------------------------------------------------
**
** Abstract: USB one byte as 2 ASCII chars
**
**
** Parameters: character to send via USB
**
**
** Returns: none
**
**
**---------------------------------------------------------------------------
*/
void
usb_byte2ascii(uint8_t tx_byte, uint8_t cantask_id) {
	uint8_t highnibble = (uint8_t) (((tx_byte >> 4) <
									 10) ? ((tx_byte >> 4) & 0x0f) + 48 : ((tx_byte >> 4) & 0x0f) +
																		  55);
	usb_putc(highnibble, cantask_id);

	uint8_t lownibble = (uint8_t) (((tx_byte & 0x0f) <
									10) ? (tx_byte & 0x0f) + 48 : (tx_byte & 0x0f) + 55);
	usb_putc(lownibble, cantask_id);
}


uint8_t ascii2byte(const uint8_t *val) {
	uint8_t temp = *val;

	if (temp > 0x60)
		temp -= 0x27;        // convert chars a-f
	else if (temp > 0x40)
		temp -= 0x07;        // convert chars A-F
	temp -= 0x30;        // convert chars 0-9

	return (uint8_t) (temp & 0x0F);
}


/*
**---------------------------------------------------------------------------
**
** Abstract: USB put SRAM string
**
**
** Parameters: pointer to string in SRAM
**
**
** Returns: none
**
**
**---------------------------------------------------------------------------
*/
void
usb_puts(uint8_t *tx_string, uint8_t cantask_id) {
	while (*tx_string)
		usb_putc(*tx_string++, cantask_id);    // send string char by char
}
