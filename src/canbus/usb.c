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

inline static void usart_read_callback(struct usart_module *const usart_module, uint8_t cantask_id);
void usart_read_callback_cantask0(struct usart_module *const usart_module);
void usart_read_callback_cantask1(struct usart_module *const usart_module);

inline static void usart_write_callback(struct usart_module *const usart_module, uint8_t cantask_id);
void usart_write_callback_cantask0(struct usart_module *const usart_module);
void usart_write_callback_cantask1(struct usart_module *const usart_module);


volatile usart_buf_t *usart_buf_can0 = NULL;
volatile usart_buf_t *usart_buf_can1 = NULL;

/**
 * writes a byte to a single wire with cpu/gpio speed
 * (for debug only, default async serial setting, 4200000 Bits/s @48Mhz CPU clock)
 * @param byte byte to write to pin
 * @param pin pin use as serial output
 */
inline static void write_byte_to_wire(uint8_t byte, uint8_t pin) {
	bool data[10] = {false};

	//set stop bit
	data[9] = true;

	//set data bits
	for (uint8_t i = 0; i <= 7; i++) {
		data[1+i] = (bool) (byte & (1<<i));
	}

	for (uint8_t i = 0; i < 10; i++) {
		if (data[i]) {
			port_pin_set_output_level(pin, true);
		} else {
			port_pin_set_output_level(pin, false);
		}
	}
}

inline static void usart_read_callback(struct usart_module *const usart_module, uint8_t cantask_id) {
	port_pin_set_output_level(PIN_PA19, true);
	port_pin_set_output_level(PIN_PA20, false);
	volatile usart_buf_t *buf;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL) {
		return;
	}
	if((buf->usart_buf_rx[buf->fill_rx][buf->rx_insert_pos]) == CR){
		taskENTER_CRITICAL();
		buf->usart_buf_rx[buf->fill_rx][buf->rx_insert_pos] = 0; // remove CR from receive buffer!
//		write_byte_to_wire((uint8_t) buf->fill_rx, PIN_PA20);
//		write_byte_to_wire((uint8_t) buf->rx_pending_map, PIN_PA20);
		buf->rx_pending_map |=  (1 << buf->fill_rx);
//		write_byte_to_wire((uint8_t) buf->rx_pending_map, PIN_PA20);
		buf->rx_fill_level++;
//		write_byte_to_wire((uint8_t) buf->rx_fill_level, PIN_PA20);
//		if (buf->rx_pending_map & (1 << ((buf->fill_rx + 1) % USB_CMD_BUF_COUNT))) {
		if (((buf->fill_rx + 1) % USB_CMD_BUF_COUNT) == buf->read_rx) {
			//if this happens the commands are not processed fast enough and to prevent overwriting the last stored
			//command we are now disabling receiving!
			buf->rx_buf_overrun = true;
			port_pin_set_output_level(PIN_PA14, true);
		}
		buf->fill_rx = (uint8_t) ((buf->fill_rx + 1) % USB_CMD_BUF_COUNT);
		buf->rx_insert_pos = -1;
		taskEXIT_CRITICAL();
	}
	buf->rx_insert_pos = (uint8_t) ((buf->rx_insert_pos + 1) % USB_CMD_BUF_SIZE);
	if (!buf->rx_buf_overrun) {
		port_pin_set_output_level(PIN_PA20, true);
		usart_read_job(usart_module, (uint16_t *) &(buf->usart_buf_rx[buf->fill_rx][buf->rx_insert_pos]));
	}
	port_pin_set_output_level(PIN_PA19, false);
}

void usart_read_callback_cantask0(struct usart_module *const usart_module) {
	usart_read_callback(usart_module, 0);
}

void usart_read_callback_cantask1(struct usart_module *const usart_module) {
	usart_read_callback(usart_module, 1);
}


inline static void usart_write_callback(struct usart_module *const usart_module, uint8_t cantask_id) {
	volatile usart_buf_t *buf;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL) {
		return;
	}
	if(buf->tx_pending_map){
//		ulog_s(" map: ");
//		xlog( (uint8_t *) &buf->tx_pending_map, 1);
//		ulog_s(" read: ");
//		xlog( (uint8_t *) &buf->read_tx, 1);
		taskENTER_CRITICAL();
		enum status_code result = usart_write_buffer_job(usart_module, (uint8_t *) &(buf->usart_buf_tx[buf->read_tx][0]), (uint16_t) (buf->buf_tx_len[buf->read_tx]));
		if (result == STATUS_OK) {
			//successfull started job
			buf->buf_tx_len[buf->read_tx] = 0;
			buf->tx_pending_map &= ~(1 << buf->read_tx);
			buf->read_tx = (uint8_t) ((buf->read_tx + 1) % USB_CMD_BUF_COUNT);
			if (buf->tx_buf_overrun) {
				buf->tx_pending_map |= (1 << buf->fill_tx);
				buf->fill_tx = (uint8_t) ((buf->fill_tx + 1) % USB_CMD_BUF_COUNT);
				buf->tx_insert_pos = 0;
				buf->tx_buf_overrun = false;
			}
		} else {

		}
		taskEXIT_CRITICAL();
	}
}

void usart_write_callback_cantask0(struct usart_module *const usart_module) {
	usart_write_callback(usart_module, 0);
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
}

void usart_write_callback_cantask1(struct usart_module *const usart_module) {
	usart_write_callback(usart_module, 1);
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
	volatile usart_buf_t *buf;
	enum_usb_return_t result = NO_CMD;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL) {
		return USB_ERROR;
	}
//	ulog_s("\r\nmap: ");
//	xlog((uint8_t *) &buf->rx_pending_map, 1);
//	ulog_s(" fill: ");
//	xlog(&buf->fill_rx, 1);
//	ulog_s(" read: ");
//	xlog(&buf->read_rx, 1);
//	taskENTER_CRITICAL();
//	if (buf->rx_pending_map & (1 << buf->read_rx)) {
	if (buf->read_rx != buf->fill_rx || buf->rx_buf_overrun) {
//		uint8_t pos = buf->read_rx;
		*cmd_buf = (uint8_t *) buf->usart_buf_rx[buf->read_rx];
		*buf_num = buf->read_rx;

//		ulog_s("\r\nBUF0: ");
//		xlog(buf->usart_buf_rx[0], CMD_BUFFER_LENGTH);
//		ulog_s("\r\nBUF1: ");
//		xlog(buf->usart_buf_rx[1], CMD_BUFFER_LENGTH);
//		c_log('n');

//		if (buf->buf_overrun) {
//			port_pin_set_output_level(PIN_PA14, false);
			/*
			 * if we had an possible overflow in the next rx buffer row, the uart read callback
			 * has not started a new reading. But we now have cleared one buffer entry so a new
			 * reading from uart can be started!
			*/
//			if (usart_read_job(usart_module, (uint16_t *) &(buf->usart_buf_rx[buf->fill_rx][buf->rx_insert_pos])) != STATUS_OK) {
//				ulog_s("\r\nrestart after buf_overrun failed!");
//			}
//			buf->buf_overrun = false;
//			result = BUF_OVERRUN;
//		} else {
			result = NEW_CMD;
//		}
	}

/*	if (buf->buf_overrun) {
		ulog_s("\r\nmap: ");
		xlog((uint8_t *) &buf->rx_pending_map, 1);
		ulog_s(" fill: ");
		xlog((uint8_t*)&buf->fill_rx, 1);
		ulog_s(" read: ");
		xlog((uint8_t*)&buf->read_rx, 1);
		ulog_s(" levl: ");
		xlog((uint8_t*)&buf->rx_fill_level, 1);
		for (uint8_t i = 0; i < USB_CMD_BUF_COUNT; i++) {
			ulog_s("\r\nBUF");
			uint8_t tmp = (uint8_t) (i+0x30);
			ulog(&tmp,1);
			ulog_s(": ");
			xlog((uint8_t*) buf->usart_buf_rx[i], CMD_BUFFER_LENGTH);
		}
		result = BUF_OVERRUN;
	}
*/
//	taskEXIT_CRITICAL();
	return result;
}

bool clear_cmd_buf(usart_module_t *usart_module, uint8_t cantask_id, uint32_t buf_num) {
	volatile usart_buf_t *buf;
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
		taskENTER_CRITICAL();
		memset((void *) buf->usart_buf_rx[buf_num], 0x00, USB_CMD_BUF_SIZE);
		buf->rx_pending_map &= ~(1 << buf_num);
		buf->rx_fill_level--;
		buf->read_rx = (uint8_t) ((buf->read_rx + 1) % USB_CMD_BUF_COUNT);

//		buf->rx_pending_map = 0;
//		for (uint32_t i = 0; i < USB_CMD_BUF_COUNT; i++) {
//			memset(buf->usart_buf_rx[i], 0x00, USB_CMD_BUF_SIZE);
//		}
		if (buf->rx_buf_overrun) {
			port_pin_set_output_level(PIN_PA14, false);
//			ulog_s(" ovr ");
			/*
			 * if we had an possible overflow in the next rx buffer row, the uart read callback
			 * has not started a new reading. But we now have cleared one buffer entry so a new
			 * reading from uart can be started!
			*/
			port_pin_set_output_level(PIN_PA20, true);
			if(usart_read_job(usart_module, (uint16_t *) &(buf->usart_buf_rx[buf->fill_rx][buf->rx_insert_pos])) != STATUS_OK) {
				ulog_s("\r\nrestart after buf_overrun failed!");
			}
			buf->rx_buf_overrun = false;
		}
		taskEXIT_CRITICAL();
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
	buf_struct->rx_buf_overrun = false;
	buf_struct->tx_buf_overrun = false;
	buf_struct->rx_insert_pos = 0;
	buf_struct->tx_insert_pos = 0;
	buf_struct->fill_rx = 0;
	buf_struct->fill_tx = 0;
	buf_struct->read_rx = 0;
	buf_struct->read_tx = 0;
	buf_struct->rx_fill_level = 0;
	buf_struct->tx_fill_level = 0;
	for (uint8_t i = 0; i < USB_CMD_BUF_COUNT; i++) {
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
	volatile usart_buf_t *buf;
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
	if (buf->tx_buf_overrun) {
		return STATUS_ERR_OVERFLOW;
	}

	enum status_code result = STATUS_OK;
	taskENTER_CRITICAL();
	if (buf->tx_insert_pos > 0){
		buf->buf_tx_len[buf->fill_tx] = buf->tx_insert_pos;
		if (((buf->fill_tx + 1) % USB_CMD_BUF_COUNT) == buf->read_tx) {
//		if (buf->tx_pending_map & ((buf->fill_tx + 1) % USB_CMD_BUF_COUNT)) {
			//if this happens the commands are not processed fast enough and we are now starting to overwrite the last stored command!
			buf->tx_buf_overrun = true;
			result = STATUS_ERR_OVERFLOW;
		} else {
			buf->tx_pending_map |= (1 << buf->fill_tx);
			buf->fill_tx = (uint8_t) ((buf->fill_tx + 1) % USB_CMD_BUF_COUNT);
			buf->tx_insert_pos = 0;
		}
	}
	taskEXIT_CRITICAL();

	//start sending data if available
	callback_func(module);
	return result;
}


/*
 * appends a char to the sequence in the buffer. does not send any data.
 *
 * @param tx_byte
 * @param cantask_id
 */
bool usb_putc(uint8_t tx_byte, uint8_t cantask_id) {
	// check which task wants to send data
	volatile usart_buf_t *buf;
	if (cantask_id == CANTASK_ID_0) {
		buf = usart_buf_can0;
	} else {
		buf = usart_buf_can1;
	}
	if (buf == NULL || buf->tx_buf_overrun) {
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
bool usb_byte2ascii(uint8_t tx_byte, uint8_t cantask_id) {
	uint8_t highnibble = (uint8_t) (((tx_byte >> 4) < 10) ? ((tx_byte >> 4) & 0x0f) + 48 : ((tx_byte >> 4) & 0x0f) + 55);
	if (!usb_putc(highnibble, cantask_id)) {
		false;
	}
	uint8_t lownibble = (uint8_t) (((tx_byte & 0x0f) < 10) ? (tx_byte & 0x0f) + 48 : (tx_byte & 0x0f) + 55);
	return usb_putc(lownibble, cantask_id);
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
bool usb_puts(uint8_t *tx_string, uint8_t cantask_id) {
	while (*tx_string) {
		if(usb_putc(*tx_string++, cantask_id)) {    // send string char by char
			return false;
		}
	}
	return true;
}
