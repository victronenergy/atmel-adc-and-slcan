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

uint8_t usb_tx_buffer0[64];
uint8_t usb_tx_buffer1[64];
uint8_t tx_buf0_pos = 0;
uint8_t tx_buf1_pos = 0;

uint8_t buffer_to_fill = 0;

volatile uint8_t received_char;
volatile bool check = false;

void usart_read_callback(struct usart_module *const usart_module) {

	check = true;
	//char *string = "read callback";
	//usart_write_buffer_job(usart_instance, (uint8_t *) string, strlen(string));
	//usart_write_buffer_job(usart_module, (uint8_t *)uart_rx_buffer, UART_RX_BUFSIZE);
	//port_pin_toggle_output_level(LEDPIN_C21_1);
	//port_pin_set_output_level(LEDPIN_C21_1, false);
}

void usart_write_callback(struct usart_module *const usart_module) {
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
}

void configure_usart_callbacks(usart_module_t *usart_instance) {
	usart_register_callback(usart_instance,
							usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(usart_instance,
							usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


bool check_usart(usart_module_t *usart_instance, uint16_t *rx_char) {
	usart_read_job(usart_instance, rx_char);
}

bool usb_getc(struct usart_module *const usart_module, uint8_t *uart_rx_buffer) {

	if (check) {
		check = false;
		//usart_read_buffer_wait(usart_module, uart_rx_buffer, 1);
		return true;
	}
	return false;

	//return uart_rx_buffer[0];

	/*
	uint8_t rx_byte;

	// check for received characters
	if ((USB_RX_PIN & _BV (USB_RXF)) != _BV (USB_RXF))	// do if RXF low
	{
		USB_TX_PORT &= ~_BV (USB_RD);	// enable RD
		asm ("NOP");
		asm ("NOP");
		asm ("NOP");
		rx_byte = USB_DATA_PIN;	// get data byte
		USB_TX_PORT |= _BV (USB_RD);	// disable RD
		return (rx_byte & 0x7F);	// return ASCII char
	}
	else

	return 0;		// return no char*/
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

void usb_send(struct usart_module *const module) {

	if (buffer_to_fill == 0) {
		buffer_to_fill = 1;
		//send buffer 0
		if (tx_buf0_pos) {
			port_pin_set_output_level(LEDPIN_C21_GREEN, LED_ACTIVE);
			usart_write_buffer_job(module, usb_tx_buffer0, tx_buf0_pos);
			tx_buf0_pos = 0;
		}
	} else {
		buffer_to_fill = 0;
		//send buffer 1
		if (tx_buf1_pos) {
			port_pin_set_output_level(LEDPIN_C21_GREEN, LED_ACTIVE);
			usart_write_buffer_job(module, usb_tx_buffer1, tx_buf1_pos);
			tx_buf1_pos = 0;
		}
	}


}

void
usb_putc(usart_module_t *usart_instance, uint8_t tx_byte) {

	if (buffer_to_fill == 0) {
		usb_tx_buffer0[tx_buf0_pos++] = tx_byte;
	}else{
		usb_tx_buffer1[tx_buf1_pos++] = tx_byte;
	}

	//usart_write_wait(usart_instance, tx_byte);

	/*
	while ((USB_TX_PIN & _BV (USB_TXE)) == _BV (USB_TXE));	// wait for Tx ready

	USB_DATA_DIR = 0xFF;	// USB data port all output
	USB_DATA_PORT = tx_byte;	// send data byte

	USB_TX_PORT |= _BV (USB_WR);	// enable WR
	asm ("NOP");
	USB_TX_PORT &= ~_BV (USB_WR);	// disable WR
	asm ("NOP");

	USB_DATA_DIR = 0x00;	// USB data port all inputs
	USB_DATA_PORT = 0xFF;	// enable data port pull-ups*/
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
usb_byte2ascii(usart_module_t *usart_instance, uint8_t tx_byte) {
	uint8_t highnibble = ((tx_byte >> 4) <
						  10) ? ((tx_byte >> 4) & 0x0f) + 48 : ((tx_byte >> 4) & 0x0f) +
															   55;
	usb_putc(usart_instance, highnibble);
	uint8_t lownibble = ((tx_byte & 0x0f) <
						 10) ? (tx_byte & 0x0f) + 48 : (tx_byte & 0x0f) + 55;
	usb_putc(usart_instance, lownibble);


	/*
    // send high nibble
    usb_putc (usart_instance, );
    // send low nibble
    usb_putc (usart_instance, );*/
}


uint8_t ascii2byte(uint8_t *val) {
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
usb_puts(usart_module_t *usart_instance, uint8_t *tx_string) {
	while (*tx_string)
		usb_putc(usart_instance, *tx_string++);    // send string char by char
}
