//*****************************************************************************
//
// Title        : AVR based USB<>CAN adaptor
// Authors      : Michael Wolf
// File Name    : 'usb.h'
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
#include "log.h"

#ifndef __USB_H__
#define __USB_H__

#define USB_CMD_BUF_SIZE 32
#define USB_CMD_BUF_COUNT 8 // must be power of 2

typedef struct {
	uint8_t usart_buf_tx[USB_CMD_BUF_COUNT][USB_CMD_BUF_SIZE];
	uint8_t usart_buf_rx[USB_CMD_BUF_COUNT][USB_CMD_BUF_SIZE];
	uint8_t fill_tx;
	uint8_t fill_rx;
	uint8_t read_tx;
	uint8_t read_rx;
	uint32_t tx_pending_map;
	uint32_t rx_pending_map;
	int8_t tx_insert_pos;
	int8_t buf_tx_len[USB_CMD_BUF_COUNT];
	int8_t rx_insert_pos;
	uint8_t rx_fill_level;
	uint8_t tx_fill_level;
	bool rx_buf_overrun;
	bool tx_buf_overrun;
} usart_buf_t;

typedef enum {
	NO_CMD = 0,
	NEW_CMD,
	BUF_OVERRUN,
	USB_ERROR,
} enum_usb_return_t;


bool usb_putc(uint8_t tx_byte, uint8_t cantask_id);
bool usb_puts(uint8_t * tx_string, uint8_t cantask_id);
enum status_code usb_send(usart_module_t *module, uint8_t cantask_id);

bool usb_byte2ascii (uint8_t tx_byte, uint8_t cantask_id);
uint8_t ascii2byte (const uint8_t *val);

enum_usb_return_t get_complete_cmd(uint8_t **cmd_buf, uint32_t *buf_num, uint8_t cantask_id);
bool clear_cmd_buf(usart_module_t *usart_module, uint8_t cantask_id, uint32_t buf_num);

void configure_usart_callbacks(usart_module_t *usart_instance, uint8_t cantask_id);
bool start_canbus_usart(usart_module_t *usart_instance, usart_buf_t *buf_struct, uint8_t cantask_id);
uint8_t* getdata_buffer(uint8_t cantask_id);

#endif // __USB_H__
