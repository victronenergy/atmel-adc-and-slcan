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
#include <samc21_usbcan/samc21_usbcan.h>
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


bool callback_pending_0 = false;
bool callback_pending_1 = false;

/*
 * usb send buffers
 * There are 2 buffers per CAN task, so 4 buffers in total.
 * The 2 buffers per task prevent problems of simultaneous writing into the buffer while sending data
 */
// buffers for can task 0
uint8_t usb_tx_buffer_0a[64];
uint8_t usb_tx_buffer_0b[64];
//current positions for can task 0
uint8_t tx_buf_0a_pos = 0;
uint8_t tx_buf_0b_pos = 0;

// buffers for can task 1
uint8_t usb_tx_buffer_1a[64];
uint8_t usb_tx_buffer_1b[64];
//current positions for can task 1
uint8_t tx_buf_1a_pos = 0;
uint8_t tx_buf_1b_pos = 0;

//constants used to indicate which buffer to fill currently
#define BUFFER_TO_FILL_A 0
#define BUFFER_TO_FILL_B 1

// indicates which buffer (A/B) is used for new chars in can task 0
uint8_t buffer_to_fill_can0 = BUFFER_TO_FILL_A;

// indicates which buffer (A/B) is used for new chars in can task 1
uint8_t buffer_to_fill_can1 = BUFFER_TO_FILL_A;

volatile bool newdata_cantask0 = false;
volatile bool newdata_cantask1 = false;

uint16_t *rx_glbl_0;
uint16_t *rx_glbl_1;

void usart_read_callback_cantask0(struct usart_module *const usart_module) {
	newdata_cantask0 = true;
}

void usart_read_callback_cantask1(struct usart_module *const usart_module) {
	newdata_cantask1 = true;
}

void usart_write_callback_cantask0(struct usart_module *const usart_module) {
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
}

void usart_write_callback_cantask1(struct usart_module *const usart_module) {
	//TODO adapt to other LED than callback0 when available
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
}

void configure_usart_callbacks(usart_module_t *usart_instance, uint8_t cantask_id) {
	if(cantask_id == CANTASK_ID_0) {
		usart_register_callback(usart_instance,
								usart_write_callback_cantask0, USART_CALLBACK_BUFFER_TRANSMITTED);
		usart_register_callback(usart_instance,
								usart_read_callback_cantask0, USART_CALLBACK_BUFFER_RECEIVED);
	}else{
		usart_register_callback(usart_instance,
								usart_write_callback_cantask1, USART_CALLBACK_BUFFER_TRANSMITTED);
		usart_register_callback(usart_instance,
								usart_read_callback_cantask1, USART_CALLBACK_BUFFER_RECEIVED);
	}
	// valid for both calls as it depends on the usart instance
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


bool usart_read_char(usart_module_t *usart_instance, uint16_t *rx_char, uint16_t *rcvd_char, uint8_t cantask_id) {
	bool retval = false;

	if (cantask_id == CANTASK_ID_0) {
		if (newdata_cantask0) {
			newdata_cantask0 = false;
			rx_glbl_0 = rx_char;
			uint16_t val = *rx_glbl_0;
			*rcvd_char = val;
			return usart_read_job(usart_instance, rx_char);
			return true;
		}
	}else{
		if (newdata_cantask1) {
			newdata_cantask1 = false;
			rx_glbl_1 = rx_char;
			uint16_t val = *rx_glbl_1;
			*rcvd_char = val;
			return usart_read_job(usart_instance, rx_char);
			return true;
		}
	}
	return retval;
}


// gets called regularly by the cantask to check for new chars via UART
bool check_usart(usart_module_t *usart_instance, uint16_t *rx_char, uint8_t cantask_id) {

		return usart_read_job(usart_instance, rx_char);
	/*
	if (cantask_id == CANTASK_ID_0) {
		if(callback_pending_0) {
			callback_pending_0 = false;
		}else{
			return usart_read_job(usart_instance, rx_char);
			callback_pending_0 = true;
		}
	}else{
		if(callback_pending_1) {
			callback_pending_1 = false;
		}else{
			return usart_read_job(usart_instance, rx_char);
			callback_pending_1 = true;
		}
	}
	return false;*/
}

// check if new data was received (set by the callback)
bool usart_new_data_available(uint8_t cantask_id) {

	if (cantask_id == CANTASK_ID_0) {
		if (newdata_cantask0) {
			newdata_cantask0 = false;
			return true;
		}
	}else{
		if (newdata_cantask1) {
			newdata_cantask1 = false;
			return true;
		}
	}
	return false;
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

void usb_send(struct usart_module *const module, uint8_t can_task_id) {

	if (can_task_id == CANTASK_ID_0) {
		// this only concerns can task 0
		if (buffer_to_fill_can0 == BUFFER_TO_FILL_A) {
			//until now, buffer A was filled. switch to buffer B for filling, so we can send the content of A
			buffer_to_fill_can0 = BUFFER_TO_FILL_B;
			//send buffer A
			if (tx_buf_0a_pos) {
				port_pin_set_output_level(LEDPIN_C21_GREEN, LED_ACTIVE);
				usart_write_buffer_job(module, usb_tx_buffer_0a, tx_buf_0a_pos);
				tx_buf_0a_pos = 0;
			}
		} else {
			buffer_to_fill_can0 = BUFFER_TO_FILL_A;
			//send buffer B
			if (tx_buf_0b_pos) {
				port_pin_set_output_level(LEDPIN_C21_GREEN, LED_ACTIVE);
				usart_write_buffer_job(module, usb_tx_buffer_0b, tx_buf_0b_pos);
				tx_buf_0b_pos = 0;
			}
		}
	} else {
		// this only concerns can task 1
		if (buffer_to_fill_can1 == BUFFER_TO_FILL_A) {
			//until now, buffer A was filled. switch to buffer B for filling, so we can send the content of A
			buffer_to_fill_can1 = BUFFER_TO_FILL_B;
			//send buffer A
			if (tx_buf_1a_pos) {
				port_pin_set_output_level(LEDPIN_C21_GREEN, LED_ACTIVE);
				usart_write_buffer_job(module, usb_tx_buffer_1a, tx_buf_1a_pos);
				tx_buf_1a_pos = 0;
			}
		} else {
			buffer_to_fill_can1 = BUFFER_TO_FILL_A;
			//send buffer B
			if (tx_buf_1b_pos) {
				port_pin_set_output_level(LEDPIN_C21_GREEN, LED_ACTIVE);
				usart_write_buffer_job(module, usb_tx_buffer_1b, tx_buf_1b_pos);
				tx_buf_1b_pos = 0;
			}
		}

	}

}


/*
 * appends a char to the sequence in the buffer. does not send any data.
 *
 * @param usart_instance
 * @param tx_byte
 * @param cantask_id
 */
void
usb_putc(uint8_t tx_byte, uint8_t cantask_id) {
	// check which task wants to send data
	if (cantask_id == CANTASK_ID_0) {
		//append to the buffer that is currently selected for filling
		if (buffer_to_fill_can0 == BUFFER_TO_FILL_A) {
			usb_tx_buffer_0a[tx_buf_0a_pos++] = tx_byte;
		} else {
			usb_tx_buffer_0b[tx_buf_0b_pos++] = tx_byte;
		}
	} else {
		//append to the buffer that is currently selected for filling
		if (buffer_to_fill_can1 == BUFFER_TO_FILL_A) {
			usb_tx_buffer_1a[tx_buf_1a_pos++] = tx_byte;
		} else {
			usb_tx_buffer_1b[tx_buf_1b_pos++] = tx_byte;
		}
	}
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


uint8_t ascii2byte(const uint8_t const *val) {
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
