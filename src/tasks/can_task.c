//
// Created by stekreis on 25.05.18.
//

#include <log.h>
#include <samc21_xplained_pro.h>
#include "can_task.h"
#include "usb.h"
#include <ctype.h>

#define PeliCANMode

#define UART_RX_BUFSIZE 32

#define SETBIT(x, y) (x |= (y))    // Set bit y in byte x
#define CLEARBIT(x, y) (x &= (~y))    // Clear bit y in byte x
#define CHECKBIT(x, y) (x & (y))    // Check bit y in byte x

// define local CAN status flags
#define CAN_INIT          0x0001    // set if CAN controller is initalized
#define MSG_WAITING       0x0002    // set if Rx message is waiting
#define BUS_ON            0x0004    // set if CAN controller is in oper mode
#define TX_BUSY           0x0008    // set if transmit is in progress

#define ERROR_WARNING     0x0400    // set if error warning was detected
#define DATA_OVERRUN      0x0800    // set if data overrun was detected
#define ERROR_PASSIVE     0x2000    // set if controller changes to error passive mode
#define ARB_LOST          0x4000    // set if arbitation lost was detected
#define BUS_ERROR         0x8000    // set if bus error was detected


#if defined (PeliCANMode)
#define LOM_Bit   1        /* listen only mode bit */
#define STM_Bit   2        /* self test mode bit */
#define AFM_Bit   3        /* acceptance filter mode bit */
#define SM_Bit    4        /* enter sleep mode bit */
#endif


// AVR bit shift macro
#define    _BV(bit)   (1 << (bit))


// TODO adapt to SAM specifics

/* address and bit definitions for the Mode & Control Register */
#define CAN_BASE    0x1100    // external RAM adress, high nibble doesn't care

#define ModeControlReg  *(uint8_t *) (CAN_BASE + 0)
#define RM_RR_Bit   0        /* reset mode (request) bit */


#define BTR0_100k 0x43
#define BTR1_100k 0x2F

// TODO END


//TODO not use globally
usart_module_t *usart_instance;

uint8_t uart_rx_buffer[UART_RX_BUFSIZE];

// CAN module, necessary to be global for CAN interrupt
// TODO make sure this does not mix up the respective CAN channels
struct can_module *can_instance_glbl;

// CAN init values for ACR, AMR and BTR registers after power up
struct {
	uint8_t acr[4];
	uint8_t amr[4];
	uint8_t btr0;
	uint8_t btr1;
	uint8_t fixed_rate;
} CAN_init_val = {{0x00, 0x00, 0x00, 0x00}, {0xff, 0xff, 0xff, 0xff}, BTR0_100k, BTR1_100k, 0};

// CAN tx message
struct {
	uint8_t format;        // Extended/Standard Frame
	uint32_t id;            // Frame ID
	uint8_t rtr;            // RTR/Data Frame
	uint8_t len;            // Data Length
	uint8_t data[8];        // Data Bytes
} CAN_tx_msg;                // length 15 byte

// CAN rx message
struct {
	uint8_t format;        // Extended/Standard Frame
	uint32_t id;            // Frame ID
	uint8_t rtr;            // RTR/Data Frame
	uint8_t len;            // Data Length
	uint8_t data[8];        // Data Bytes
} CAN_rx_msg;                  // length 15 byte/each


volatile uint16_t CAN_flags;
volatile uint8_t last_ecr;
volatile uint8_t last_alc;


uint8_t data[8] = {0x01};

/*
 * Prototypes
 */
void vCanTask(void *pvParameters);

void init_can_mem(void);

void usart_read_callback(struct usart_module *usart_module);

void usart_write_callback(struct usart_module *usart_module);

void configure_usart_callbacks(void);

uint8_t exec_usb_cmd(uint8_t *cmd_buf);

static void can_send_standard_message(struct can_module *can_instance, uint32_t id_value, uint8_t *data);


uint8_t init_CAN_mictronics(void);

uint8_t transmit_CAN(struct can_module *can_instance, uint32_t id_value, uint8_t *data);

uint8_t read_CAN_reg(uint8_t reg);

void write_CAN_reg(uint8_t cmd_len, uint8_t tmp_regdata);


// TODO implement
uint8_t init_CAN_mictronics(void) {

	// clear CAN error flags
	CAN_flags &= 0x00FF;
	return CR;
}

uint8_t transmit_CAN(struct can_module *const can_instance, uint32_t id_value, uint8_t *data) {
	ulog_s("TRANSMIT_CAN");
	can_send_standard_message(can_instance, id_value, data);
	return CR;
}

//TODO implement.
// original: read register content from SJA1000
uint8_t read_CAN_reg(uint8_t reg) {
	return 0;
}

void write_CAN_reg(uint8_t cmd_len, uint8_t tmp_regdata) {

}
// TODO END



static void can_send_standard_message(struct can_module *const can_instance, uint32_t id_value, uint8_t *data) {
	ulog_s("sending message via CAN");
	uint32_t i;
	struct can_tx_element tx_element;
	can_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	tx_element.T1.bit.DLC = 8;
	for (i = 0; i < 8; i++) {
		tx_element.data[i] = *data;
		data++;
	}
	can_set_tx_buffer_element(can_instance, &tx_element,
							  CAN_TX_BUFFER_INDEX);
	can_tx_transfer_request(can_instance, 1 << CAN_TX_BUFFER_INDEX);
}

void usart_read_callback(struct usart_module *const usart_module) {
	char *string = "read callback";
	usart_write_buffer_job(usart_instance, (uint8_t *) string, strlen(string));
	//usart_write_buffer_job(usart_instance, (uint8_t *)uart_rx_buffer, UART_RX_BUFSIZE);
	//port_pin_toggle_output_level(LED_0_PIN);


}

void usart_write_callback(struct usart_module *const usart_module) {
	//port_pin_toggle_output_level(LED_0_PIN);
}

void configure_usart_callbacks(void) {
	usart_register_callback(usart_instance,
							usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(usart_instance,
							usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(usart_instance,
						  USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

void vCanTask(void *pvParameters) {
	ulog_s("cantask begin loop\r\n");

	vTaskDelay((const TickType_t) 1000);

	uint8_t usb_rx_char;

	uint8_t cmd_buf[CMD_BUFFER_LENGTH];
	uint8_t buf_ind = 0;

	uint8_t i;

	init_CAN_mictronics();

	init_can_mem();

	configure_usart_callbacks();

	//TODO prepare CAN interface

	ulog_s("\r\n");

	can_enable_interrupt(can_instance_glbl, CAN_RX_FIFO_0_NEW_MESSAGE);
	can_enable_interrupt(can_instance_glbl, CAN_RX_FIFO_1_NEW_MESSAGE);

	for (;;) {
		vTaskDelay((const TickType_t) 100);
		//port_pin_toggle_output_level(LED_0_PIN);

		// TODO perform CAN task

		//usart_read_buffer_wait(usart_instance, uart_rx_buffer, 1);

		//uint8_t string[] = "senddatatopc\r\n";
		//usart_write_buffer_wait(usart_instance, string, sizeof(string));

		if (CHECKBIT(CAN_flags, MSG_WAITING)) {
			ulog_s("data received");
/*
			volatile uint32_t status, rx_buffer_index;
			status = can_read_interrupt_status(can_instance_glbl);
			if (status & CAN_RX_FIFO_1_NEW_MESSAGE) {
				can_clear_interrupt_status(can_instance_glbl, CAN_RX_FIFO_1_NEW_MESSAGE);
				ulog_s("somesing");
			}
*/

			// check frame format
			if (!CAN_rx_msg.format) {    // Standart Frame
				if (!CAN_rx_msg.rtr) {
					usb_putc(usart_instance, SEND_11BIT_ID);
				}        // send command tag
				else {
					usb_putc(usart_instance, SEND_R11BIT_ID);
				}

				// send high byte of ID
				if (((CAN_rx_msg.id >> 8) & 0x0F) < 10)
					usb_putc(usart_instance, (uint8_t) (((CAN_rx_msg.id >> 8) & 0x0F) + 48));
				else
					usb_putc(usart_instance, (uint8_t) (((CAN_rx_msg.id >> 8) & 0x0F) + 55));
				// send low byte of ID
				usb_byte2ascii(usart_instance, (uint8_t) (CAN_rx_msg.id & 0xFF));
			} else {        // Extented Frame
				if (!CAN_rx_msg.rtr) {
					usb_putc(usart_instance, SEND_29BIT_ID);
				}        // send command tag
				else {
					usb_putc(usart_instance, SEND_R29BIT_ID);
				}
				// send ID bytes
				usb_byte2ascii(usart_instance, (uint8_t) ((CAN_rx_msg.id >> 24) & 0xFF));
				usb_byte2ascii(usart_instance, (uint8_t) ((CAN_rx_msg.id >> 16) & 0xFF));
				usb_byte2ascii(usart_instance, (uint8_t) ((CAN_rx_msg.id >> 8) & 0xFF));
				usb_byte2ascii(usart_instance, (uint8_t) (CAN_rx_msg.id & 0xFF));
			}
			// send data length code
			usb_putc(usart_instance, (uint8_t) (CAN_rx_msg.len + '0'));
			if (!CAN_rx_msg.rtr) {    // send data only if no remote frame request
				// send data bytes
				for (i = 0; i < CAN_rx_msg.len; i++)
					usb_byte2ascii(usart_instance, CAN_rx_msg.data[i]);
			}
			/*	TODO clarify: TIMESTAMP required at all?
			// send time stamp if required
			if (ram_timestamp_status != 0) {
				usb_byte2ascii((uint8_t) (timestamp >> 8));
				usb_byte2ascii((uint8_t) timestamp);
			}
			 */
			// send end tag
			usb_putc(usart_instance, CR);
			CLEARBIT(CAN_flags, MSG_WAITING);
		}

		//xlog(uart_rx_buffer, UART_RX_BUFSIZE);
		ulog_s("\r\n");
		// read char from USB
		usb_rx_char = usb_getc(usart_instance, uart_rx_buffer);    // check for new chars from USB
		if (usb_rx_char == CR)    // check for end of command
		{
			// Execute USB command and return status to terminal
			usb_putc(usart_instance, exec_usb_cmd(cmd_buf));

			// flush command buffer
			for (buf_ind = 0; buf_ind < CMD_BUFFER_LENGTH; buf_ind++)
				cmd_buf[buf_ind] = 0x00;

			buf_ind = 0;    // point to start of command
		} else if (usb_rx_char != 0)    // store new char in buffer
		{
			cmd_buf[buf_ind] = usb_rx_char;    // store char
			// check for buffer overflow
			if (buf_ind < sizeof(cmd_buf) - 1)
				buf_ind++;
		}
		flush_clog();
	}
}

#pragma clang diagnostic pop


uint8_t exec_usb_cmd(uint8_t *cmd_buf) {
	ulog_s("EXEC USB CMD\r\n");

	uint8_t cmd_len = strlen((char *) cmd_buf);    // get command length

	uint8_t *cmd_buf_pntr = &(*cmd_buf);    // point to start of received string
	cmd_buf_pntr++;        // skip command identifier
	// check if all chars are valid hex chars
	while (*cmd_buf_pntr) {
		if (!isxdigit(*cmd_buf_pntr))
			return ERROR;
		++cmd_buf_pntr;
	}
	cmd_buf_pntr = &(*cmd_buf);    // reset pointer

	uint8_t *tmp_pntr;        // temporary pointer for ACR/AMR
	uint8_t tmp_regdata;    // temporary used for register data

	switch (*cmd_buf_pntr) {
		// get serial number
		case GET_SERIAL:
			ulog_s("request serial");
			usb_putc(usart_instance, GET_SERIAL);
			usb_puts(usart_instance, (uint8_t *) SERIAL);
			return CR;

			// get hard- and software version
		case GET_VERSION:
			ulog_s("request version");
			usb_putc(usart_instance, GET_VERSION);
			usb_byte2ascii(usart_instance, HW_VER);
			usb_byte2ascii(usart_instance, SW_VER);
			return CR;

			// get only software version
		case GET_SW_VERSION:
			ulog_s("request sw version");
			usb_putc(usart_instance, GET_SW_VERSION);
			usb_byte2ascii(usart_instance, SW_VER_MAJOR);
			usb_byte2ascii(usart_instance, SW_VER_MINOR);
			return CR;
/*
 * TODO clarify if actually needed
			// toggle time stamp option
		case TIME_STAMP:
			ulog_s("toggle time stamp option");
			// read stored status
			ram_timestamp_status = eeprom_read_byte(&ee_timestamp_status);
			// toggle status
			if (ram_timestamp_status != 0)
				ram_timestamp_status = 0;    // disable time stamp
			else {
				ram_timestamp_status = 0xA5;    // enable time stamp
				timestamp = 0;    // reset time stamp counter
			}
			// store new status
			eeprom_write_byte(&ee_timestamp_status, ram_timestamp_status);
			return CR;
*/
			// read status flag
		case READ_STATUS:
			ulog_s("request status flag");
			// check if CAN controller is in reset mode
			if (!CHECKBIT(CAN_flags, BUS_ON))
				return ERROR;

			usb_putc(usart_instance, READ_STATUS);
			usb_byte2ascii(usart_instance, (uint8_t) (CAN_flags >> 8));

			// turn off Bus Error indication
			// TODO replace: LED_PORT &= ~_BV(LED_RED);
			ulog_s("bus error indication turned off.\r\n");

			// reset error flags
			CAN_flags &= 0x00FF;
			ulog_s("error flags were reset.\r\n");
			return CR;

			// set AMR
		case SET_AMR:
			// set ACR
		case SET_ACR:
			ulog_s("set AMR/ACR");
			// check valid cmd length and if CAN was initialized before
			if (cmd_len != 9)
				return ERROR;
			// check if CAN controller is in reset mode
			if (CHECKBIT(CAN_flags, BUS_ON))
				return ERROR;

			// assign pointer to AMR or ACR values depending on command
			if (*cmd_buf_pntr == SET_AMR)
				tmp_pntr = CAN_init_val.amr;
			else
				tmp_pntr = CAN_init_val.acr;

			// store AMR or ACR values
			*tmp_pntr = ascii2byte(++cmd_buf_pntr);
			*tmp_pntr <<= 4;
			*(tmp_pntr++) |= ascii2byte(++cmd_buf_pntr);
			*tmp_pntr = ascii2byte(++cmd_buf_pntr);
			*tmp_pntr <<= 4;
			*(tmp_pntr++) |= ascii2byte(++cmd_buf_pntr);
			*tmp_pntr = ascii2byte(++cmd_buf_pntr);
			*tmp_pntr <<= 4;
			*(tmp_pntr++) |= ascii2byte(++cmd_buf_pntr);
			*tmp_pntr = ascii2byte(++cmd_buf_pntr);
			*tmp_pntr <<= 4;
			*tmp_pntr |= ascii2byte(++cmd_buf_pntr);
			// init CAN controller with new values
			return init_CAN_mictronics();

			// set bitrate via BTR
		case SET_BTR:
			// set fix bitrate
		case SET_BITRATE:

			//TODO remove these 2 cmds, used for testing only
			*(cmd_buf_pntr + 1) = 5;
			cmd_len = 2;

			ulog_s("set bitrate");
			if ((cmd_len != 5) && (cmd_len != 2)) {
				ulog_s("invalid cmd length");
				return ERROR;    // check valid cmd length
			}
			// check if CAN controller is in reset mode
			if (CHECKBIT(CAN_flags, BUS_ON)) {
				ulog_s("controller is in reset");
				return ERROR;
			}

			// store user or fixed bit rate
			if (*cmd_buf_pntr == SET_BTR) {
				CAN_init_val.btr0 = ascii2byte(++cmd_buf_pntr);
				CAN_init_val.btr0 <<= 4;
				CAN_init_val.btr0 |= ascii2byte(++cmd_buf_pntr);
				CAN_init_val.btr1 = ascii2byte(++cmd_buf_pntr);
				CAN_init_val.btr1 <<= 4;
				CAN_init_val.btr1 |= ascii2byte(++cmd_buf_pntr);
				CAN_init_val.fixed_rate = 0;
			} else {
				CAN_init_val.fixed_rate = *(++cmd_buf_pntr);
			}
			// init CAN controller
			SETBIT(CAN_flags, CAN_INIT);    // indicate initialized controller
			return init_CAN_mictronics();

			// open CAN channel
		case OPEN_CAN_CHAN:
			ulog_s("open CAN channel");
			// return error if controller is not initialized or already open
			if (!CHECKBIT(CAN_flags, CAN_INIT)) {
				ulog_s("controller not initialized or already open");
				return ERROR;
			}

			// check if CAN controller is in reset mode
			if (CHECKBIT(CAN_flags, BUS_ON)) {
				ulog_s("controller in reset mode");
				return ERROR;
			}

/* TODO comment in
			// switch to oper mode
			do {
#if defined(ENABLE_SELFTEST)
				ModeControlReg = _BV (STM_Bit);
#else
				ModeControlReg &= ~_BV(RM_RR_Bit);
#endif
			} while ((ModeControlReg & _BV(RM_RR_Bit)) == _BV(RM_RR_Bit));
			*/
			SETBIT(CAN_flags, BUS_ON);
			return CR;

			// close CAN channel
		case CLOSE_CAN_CHAN:
			ulog_s("close CAN channel");
			// check if CAN controller is in reset mode
			if (!CHECKBIT(CAN_flags, BUS_ON))
				return ERROR;

			// switch to reset mode
			do {        // do as long as RM_RR_Bit is not set
#if defined(ENABLE_SELFTEST)
				ModeControlReg = _BV (RM_RR_Bit) | _BV (STM_Bit);
#else
				ModeControlReg = _BV(RM_RR_Bit);
#endif
			} while ((ModeControlReg & _BV(RM_RR_Bit)) != _BV(RM_RR_Bit));
			CLEARBIT(CAN_flags, BUS_ON);
			return CR;

			// send R11bit ID message
		case SEND_R11BIT_ID:
			ulog_s("send 11bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!CHECKBIT(CAN_flags, BUS_ON) || CHECKBIT(CAN_flags, TX_BUSY))
				return ERROR;
			// check valid cmd length (only 5 bytes for RTR)
			if (cmd_len != 5)
				return ERROR;

			CAN_tx_msg.rtr = 1;    // remote transmission request

			// store std. frame format
			CAN_tx_msg.format = 0;
			// store ID
			CAN_tx_msg.id = ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);

			// if transmit buffer was empty send message

			return transmit_CAN(can_instance_glbl, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

		case SEND_11BIT_ID:
			ulog_s("send 11bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!CHECKBIT(CAN_flags, BUS_ON) || CHECKBIT(CAN_flags, TX_BUSY)) {
				xlog(&CAN_flags, 2);
				ulog_s("CAN controller in reset mode or busy");
				return ERROR;
			}

			if ((cmd_len < 5) || (cmd_len > 21)) {
				ulog_s("invalid cmd length");
				return ERROR;    // check valid cmd length
			}

			CAN_tx_msg.rtr = 0;    // no remote transmission request

			// store std. frame format
			CAN_tx_msg.format = 0;
			// store ID
			CAN_tx_msg.id = ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			// check number of data bytes supplied against data lenght byte
			if (CAN_tx_msg.len != ((cmd_len - 5) / 2))
				return ERROR;

			// check for valid length
			if (CAN_tx_msg.len > 8)
				return ERROR;
			else {        // store data
				// cmd_len is no longer needed, so we can use it as counter here
				for (cmd_len = 0; cmd_len < CAN_tx_msg.len; cmd_len++) {
					cmd_buf_pntr++;
					CAN_tx_msg.data[cmd_len] = ascii2byte(cmd_buf_pntr);
					CAN_tx_msg.data[cmd_len] <<= 4;
					cmd_buf_pntr++;
					CAN_tx_msg.data[cmd_len] += ascii2byte(cmd_buf_pntr);
				}
			}
			ulog_s("sending trending pending");
			// if transmit buffer was empty send message
			return transmit_CAN(can_instance_glbl, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

			// send 29bit ID message
		case SEND_R29BIT_ID:
			ulog_s("send R29bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!CHECKBIT(CAN_flags, BUS_ON) || CHECKBIT(CAN_flags, TX_BUSY))
				return ERROR;

			if (cmd_len != 10)
				return ERROR;    // check valid cmd length

			CAN_tx_msg.rtr = 1;    // remote transmission request

			// store ext. frame format
			CAN_tx_msg.format = 1;
			// store ID
			CAN_tx_msg.id = ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			// if transmit buffer was empty send message
			return transmit_CAN(can_instance_glbl, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

		case SEND_29BIT_ID:
			ulog_s("send 29bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!CHECKBIT(CAN_flags, BUS_ON) || CHECKBIT(CAN_flags, TX_BUSY))
				return ERROR;

			if ((cmd_len < 10) || (cmd_len > 26))
				return ERROR;    // check valid cmd length

			CAN_tx_msg.rtr = 0;    // no remote transmission request

			// store ext. frame format
			CAN_tx_msg.format = 1;
			// store ID
			CAN_tx_msg.id = ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			// check number of data bytes supplied against data lenght byte
			if (CAN_tx_msg.len != ((cmd_len - 10) / 2))
				return ERROR;

			// check for valid length
			if (CAN_tx_msg.len > 8)
				return ERROR;
			else {        // store data
				// cmd_len is no longer needed, so we can use it as counter here
				for (cmd_len = 0; cmd_len < CAN_tx_msg.len; cmd_len++) {
					cmd_buf_pntr++;
					CAN_tx_msg.data[cmd_len] = ascii2byte(cmd_buf_pntr);
					CAN_tx_msg.data[cmd_len] <<= 4;
					cmd_buf_pntr++;
					CAN_tx_msg.data[cmd_len] += ascii2byte(cmd_buf_pntr);
				}
			}
			// if transmit buffer was empty send message
			return transmit_CAN(can_instance_glbl, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

			// read Error Capture Register
			// read Arbitration Lost Register
		case READ_ECR:
		case READ_ALCR:
			ulog_s("read ECR/ALCR");
			// check if CAN controller is in reset mode
			if (!CHECKBIT(CAN_flags, BUS_ON))
				return ERROR;

			if (*cmd_buf_pntr == READ_ECR) {
				usb_putc(usart_instance, READ_ECR);
				usb_byte2ascii(usart_instance, last_ecr);
			} else {
				usb_putc(usart_instance, READ_ALCR);
				usb_byte2ascii(usart_instance, last_alc);
			}
			return CR;

			// read SJA1000 register
		case READ_REG:
			ulog_s("read SJA1000 register");
			if (cmd_len != 3)
				return ERROR;    // check valid cmd length
			// cmd_len is no longer needed, so we can use it as buffer
			// get register number
			cmd_len = ascii2byte(++cmd_buf_pntr);
			cmd_len <<= 4;
			cmd_len |= ascii2byte(++cmd_buf_pntr);
			usb_putc(usart_instance, READ_REG);
			usb_byte2ascii(usart_instance, read_CAN_reg(cmd_len));
			return CR;

			// write SJA1000 register
		case WRITE_REG:
			ulog_s("write SJA1000 register");
			if (cmd_len != 5)
				return ERROR;    // check valid cmd length

			// cmd_len is no longer needed, so we can use it as buffer
			// get register number
			cmd_len = ascii2byte(++cmd_buf_pntr);
			cmd_len <<= 4;
			cmd_len |= ascii2byte(++cmd_buf_pntr);
			// get register data
			tmp_regdata = ascii2byte(++cmd_buf_pntr);
			tmp_regdata <<= 4;
			tmp_regdata |= ascii2byte(++cmd_buf_pntr);
			write_CAN_reg(cmd_len, tmp_regdata);
			return CR;

		case LISTEN_ONLY:
			ulog_s("listen only mode");
			// return error if controller is not initialized or already open
			if (!CHECKBIT(CAN_flags, CAN_INIT))
				ulog_s("can not initlzd");
			return ERROR;

			// check if CAN controller is in reset mode
			if (CHECKBIT(CAN_flags, BUS_ON))
				ulog_s("can ctrl in reset");
			return ERROR;

			// switch to listen only mode
			/*	do {
					ModeControlReg = _BV(LOM_Bit);
				} while ((ModeControlReg & _BV(RM_RR_Bit)) == _BV(RM_RR_Bit));*/
			SETBIT(CAN_flags, BUS_ON);
			return CR;

			//TODO remove the following case
		case 'Q':
			ulog_s("Q testing case\r\n");
			can_send_standard_message(can_instance_glbl, CAN_RX_STANDARD_FILTER_ID_0, data);


			return CR;

			// end with error on unknown commands
		default:
			ulog_s("unknown command");
			return ERROR;
	}                // end switch

	// we should never reach this return
	return ERROR;
}                // end exec_usb_cmd



void init_can_mem() {
	/* Initialize the memory. */
	for (uint32_t i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
		tx_message_0[i] = (uint8_t) i;
		tx_message_1[i] = (uint8_t) (i + 0x80);
	}
}


void CAN0_Handler(void) {

	SETBIT(CAN_flags, MSG_WAITING);

	volatile uint32_t status, rx_buffer_index;
	status = can_read_interrupt_status(can_instance_glbl);
	if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
		can_clear_interrupt_status(can_instance_glbl, CAN_RX_FIFO_0_NEW_MESSAGE);
		can_get_rx_fifo_0_element(can_instance_glbl, &rx_element_fifo_0,
								  standard_receive_index);
		can_rx_fifo_acknowledge(can_instance_glbl, 0,
								standard_receive_index);
		standard_receive_index++;
		if (standard_receive_index == CONF_CAN0_RX_FIFO_0_NUM) {
			standard_receive_index = 0;
		}
		ulog_s("\n\r Standard message received in FIFO 0. The received data is: \r\n");
		xlog(rx_element_fifo_0.data, (uint8_t) (rx_element_fifo_0.R1.bit.DLC));
		ulog_s("\r\n\r\n");
	}
	if (status & CAN_RX_FIFO_1_NEW_MESSAGE) {
		can_clear_interrupt_status(can_instance_glbl, CAN_RX_FIFO_1_NEW_MESSAGE);
		can_get_rx_fifo_1_element(can_instance_glbl, &rx_element_fifo_1,
								  extended_receive_index);
		can_rx_fifo_acknowledge(can_instance_glbl, 0,
								extended_receive_index);
		extended_receive_index++;
		if (extended_receive_index == CONF_CAN0_RX_FIFO_1_NUM) {
			extended_receive_index = 0;
		}
		ulog_s("\n\r Extended message received in FIFO 1. The received data is: \r\n");
		xlog(rx_element_fifo_1.data, (uint8_t) (rx_element_fifo_1.R1.bit.DLC));
		ulog_s("\r\n\r\n");
	}
	if ((status & CAN_PROTOCOL_ERROR_ARBITRATION)
		|| (status & CAN_PROTOCOL_ERROR_DATA)) {
		can_clear_interrupt_status(can_instance_glbl, CAN_PROTOCOL_ERROR_ARBITRATION
												  | CAN_PROTOCOL_ERROR_DATA);
		ulog_s("Protocol error, please double check the clock in two boards. \r\n\r\n");
	}

}


TaskHandle_t vCreateCanTask(usart_module_t *p_usart_instance, struct can_module *p_can_instance) {

	usart_instance = p_usart_instance;
	can_instance_glbl = p_can_instance;

	ulog_s("creating CAN Task...\r\n");

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vCanTask,       /* Function that implements the task. */
			"canTASK",          /* Text name for the task. */
			250,      /* Stack size in words, not bytes. */
			NULL,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY + 5,/* Priority at which the task is created. */
			&xHandle);      /* Used to pass out the created task's handle. */

	if (xReturned == pdPASS) {
		ulog_s("\r\nsuccessfully created CAN Task\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		//vTaskDelete( xHandle );
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;


}

