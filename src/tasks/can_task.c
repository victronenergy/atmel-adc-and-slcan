//
// Created by stekreis on 25.05.18.
//

#include <log.h>
#include "can_task.h"
#include "usb.h"
#include <ctype.h>
#include <samc21_usbcan/samc21_usbcan.h>
#include <cmsis/samc21/include/component/can.h>

#define PeliCANMode


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

#define STANDARD_FRAME 0
#define EXTENDED_FRAME 1


// CAN init values for ACR, AMR and BTR registers after power up
struct CAN_init_val_struct{
	uint8_t acr[4];
	uint8_t amr[4];
	uint8_t btr0;
	uint8_t btr1;
	uint8_t fixed_rate;
};

// CAN tx message
struct CAN_tx_msg_struct{
	uint8_t format;        // Extended/Standard Frame
	uint32_t id;            // Frame ID
	uint8_t rtr;            // RTR/Data Frame
	uint8_t len;            // Data Length
	uint8_t data[8];        // Data Bytes
} ;                // length 15 byte

// CAN rx message
struct CAN_rx_msg_struct{
	uint8_t format;        // Extended/Standard Frame
	uint32_t id;            // Frame ID
	uint8_t rtr;            // RTR/Data Frame
	uint8_t len;            // Data Length
	uint8_t data[8];        // Data Bytes
};                  // length 15 byte/each



volatile uint8_t last_ecr;
volatile uint8_t last_alc;




/*
 * Prototypes
 */
void vCanTask(void *pvParameters);

uint8_t exec_usb_cmd(usart_module_t *usart_instance, struct can_module *can_instance, uint8_t *cmd_buf, uint16_t *CAN_flags, struct CAN_init_val_struct *CAN_init_val,uint8_t cantask_id);

void can_send_standard_message(struct can_module *can_instance, uint32_t id_value, uint8_t *data);


uint8_t reset_can_errorflags(uint16_t *CAN_flags);

uint8_t transmit_CAN(struct can_module *can_instance, uint32_t id_value, uint8_t *data);

uint8_t read_CAN_reg(uint8_t reg);

void write_CAN_reg(uint8_t cmd_len, uint8_t tmp_regdata);


// CAN status bit access
void setbit(uint16_t *res, uint8_t bitpos);
void clearbit(uint16_t *res, uint8_t bitpos);
uint16_t checkbit(uint16_t *res, uint8_t bitpos);

void setbit(uint16_t *res, uint8_t bitpos){
	*res |= bitpos;
}

void clearbit(uint16_t *res, uint8_t bitpos){
	*res &= ~bitpos;
}

uint16_t checkbit(uint16_t *res, uint8_t bitpos){
	return *res & bitpos;
}


// TODO implement
uint8_t reset_can_errorflags(uint16_t *CAN_flags) {

	// clear CAN error flags
	*CAN_flags &= 0x00FF;
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

void can_send_standard_message(struct can_module *const can_instance, uint32_t id_value, uint8_t *data) {
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




#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

void set_can_init_values(struct CAN_init_val_struct *can_init_val_struct, uint16_t CAN_flags){
	reset_can_errorflags(&CAN_flags);

	memset(can_init_val_struct->acr, 0x00, 4);
	memset(can_init_val_struct->amr, 0xff, 4);
	can_init_val_struct->btr0 = BTR0_100k;
	can_init_val_struct->btr1 = BTR1_100k;
	can_init_val_struct->fixed_rate = 0x00;
}


bool adapt_rx_can_msg(struct can_module *const module_inst, struct CAN_rx_msg_struct *CAN_rx_msg, uint16_t *CAN_flags) {

	//FIFO buffers status
	volatile CAN_RXF0S_Type fifo0_status = module_inst->hw->RXF0S;
	volatile CAN_RXF1S_Type fifo1_status = module_inst->hw->RXF1S;

	//FIFO buffers fill level
	uint32_t fifo0_fill_level = (uint32_t) fifo0_status.bit.F0FL;
	uint32_t fifo1_fill_level = (uint32_t) fifo1_status.bit.F1FL;

	// check for errors
	volatile uint32_t status = can_read_interrupt_status(module_inst);
	if ((status & CAN_PROTOCOL_ERROR_ARBITRATION)
		|| (status & CAN_PROTOCOL_ERROR_DATA)) {
		port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		can_clear_interrupt_status(module_inst, CAN_PROTOCOL_ERROR_ARBITRATION
												| CAN_PROTOCOL_ERROR_DATA);
		//ulog_s("Protocol error, please double check the clock in two boards. \r\n\r\n");
	}


	// TODO: we only have 1 struct to return either standard or extended messages.
	// currently, if both a standard and an extended msg are received at the same time, the standard msg is overwritten

	// used to check whether standard AND extended msg are processed
	bool checkboth = false;

	// fifo0 is used for standard messages
	if(fifo0_fill_level) {
		// fifo0 contains (new) data

		checkboth = true;

		setbit(CAN_flags, MSG_WAITING);

		if(fifo0_status.bit.RF0L){
			ulog_s("message lost!F0\r\n");
		}
		if(fifo0_status.bit.F0F){
			ulog_s("FIFO full!F0\r\n");
		}


		uint32_t fifo0_getindex = (uint32_t) fifo0_status.bit.F0GI;	//returns the current index of the receive buffer
		struct can_rx_element_fifo_0 rx_element_fifo_0;

		can_get_rx_fifo_0_element(module_inst, &rx_element_fifo_0,
								  fifo0_getindex);
		can_rx_fifo_acknowledge(module_inst, 0,
								fifo0_getindex);

		CAN_rx_msg->format = STANDARD_FRAME;
		CAN_rx_msg->id = rx_element_fifo_0.R0.bit.ID;
		CAN_rx_msg->rtr = (uint8_t) rx_element_fifo_0.R0.bit.RTR;
		CAN_rx_msg->len = rx_element_fifo_0.R1.bit.DLC;
		memcpy(CAN_rx_msg->data, rx_element_fifo_0.data, CAN_rx_msg->len);
	}

	// fifo1 is used for extended messages
	if (fifo1_fill_level) {
		// fifo1 contains (new) data

		if(checkboth){
			ulog_s("received both standard and extended data.\r\nCaution: standard data is currently overwritten");
		}
		setbit(CAN_flags, MSG_WAITING);

		if(fifo1_status.bit.RF1L){
			ulog_s("message lost!F1\r\n");
		}
		if(fifo1_status.bit.F1F){
			ulog_s("FIFO full!F1\r\n");
		}

		uint32_t fifo1_getindex = (uint32_t) fifo1_status.bit.F1GI;	//returns the current index of the receive buffer
		struct can_rx_element_fifo_1 rx_element_fifo_1;
		can_get_rx_fifo_1_element(module_inst, &rx_element_fifo_1,
								  fifo1_getindex);

		can_rx_fifo_acknowledge(module_inst, 1,
								fifo1_getindex);

		CAN_rx_msg->format = EXTENDED_FRAME;
		CAN_rx_msg->id = rx_element_fifo_1.R0.bit.ID;
		CAN_rx_msg->rtr = (uint8_t) rx_element_fifo_1.R0.bit.RTR;
		CAN_rx_msg->len = rx_element_fifo_1.R1.bit.DLC;
		memcpy(CAN_rx_msg->data, rx_element_fifo_1.data, CAN_rx_msg->len);
	}
}

/*
 * slcan cmds
 *
 *	sudo slcand -o -c -s5 -S 921600 /dev/ttyUSB0 can0
 *	sudo slcan_attach /dev/ttyUSB0
 *	sudo ifconfig can0 up
 *
 *
 */

/**
 * CanTask handles data incoming from the CAN interface(s) as well as via the UART interface
 *
 *
 * @param pvParameters
 */
void vCanTask(void *pvParameters) {

	vTaskDelay((const TickType_t) 10);

	// CAN status flags
	uint16_t CAN_flags = 0;

	//task parameters (uart+can instances, task id)
	cantask_params *params = (cantask_params *) pvParameters;
	struct can_module *can_module = params->can_instance;
	usart_module_t *usart_instance = params->usart_instance;
	uint8_t cantask_id = params->task_id;

	// later stores single (cmd) byte received via USB
	uint16_t usb_rx_char;

	// cmd buffer stores cmd data to send via USB
	uint8_t cmd_buf[CMD_BUFFER_LENGTH];
	memset(cmd_buf, 0x00, CMD_BUFFER_LENGTH);
	uint8_t buf_ind = 0;


	//initially both LEDs off
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
	port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);

	configure_usart_callbacks(usart_instance, cantask_id);

	struct CAN_rx_msg_struct CAN_rx_msg;

	//initialize CAN error status
	struct CAN_init_val_struct CAN_init_val;
	set_can_init_values(&CAN_init_val, &CAN_flags);


	induce_callback(usart_instance, &usb_rx_char, cantask_id);

	for (;;) {

		//TODO remove, should be only set via OPEN_CAN cmd
		//setbit(&CAN_flags, BUS_ON);

		port_pin_toggle_output_level(PIN_PA11);

		//receive data from CAN bus

		adapt_rx_can_msg(can_module, &CAN_rx_msg, &CAN_flags);

		if (checkbit(&CAN_flags, MSG_WAITING)) {
			// check frame format
			if (!CAN_rx_msg.format) {    // Standard Frame
				if (!CAN_rx_msg.rtr) {
					usb_putc(SEND_11BIT_ID, cantask_id);
				}        // send command tag
				else {
					usb_putc(SEND_R11BIT_ID, cantask_id);
				}
				// send high byte of ID
				if (((CAN_rx_msg.id >> 8) & 0x0F) < 10)
					usb_putc((uint8_t) (((CAN_rx_msg.id >> 8) & 0x0F) + 48), cantask_id);
				else
					usb_putc((uint8_t) (((CAN_rx_msg.id >> 8) & 0x0F) + 55), cantask_id);
				// send low byte of ID
				usb_byte2ascii((uint8_t) (CAN_rx_msg.id & 0xFF), cantask_id);
			} else {        // Extended Frame
				if (!CAN_rx_msg.rtr) {
					usb_putc(SEND_29BIT_ID, cantask_id);
				}        // send command tag
				else {
					usb_putc(SEND_R29BIT_ID, cantask_id);
				}
				// send ID bytes
				usb_byte2ascii((uint8_t) ((CAN_rx_msg.id >> 24) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) ((CAN_rx_msg.id >> 16) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) ((CAN_rx_msg.id >> 8) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) (CAN_rx_msg.id & 0xFF), cantask_id);
			}
			// send data length code
			usb_putc((uint8_t) (CAN_rx_msg.len + '0'), cantask_id);
			if (!CAN_rx_msg.rtr) {    // send data only if no remote frame request
				// send data bytes
				for (uint8_t can_rxmsg_pos = 0; can_rxmsg_pos < CAN_rx_msg.len; can_rxmsg_pos++)
					usb_byte2ascii(CAN_rx_msg.data[can_rxmsg_pos],cantask_id);
			}
			/*	TODO clarify: TIMESTAMP required at all?
			// send time stamp if required
			if (ram_timestamp_status != 0) {
				usb_byte2ascii((uint8_t) (timestamp >> 8));
				usb_byte2ascii((uint8_t) timestamp);
			}
			 */
			// send end tag
			usb_putc(CR,cantask_id);
			clearbit(&CAN_flags, MSG_WAITING);
		}



		bool new_cmd_available = get_complete_cmd(cmd_buf, cantask_id);


		if (new_cmd_available) {
			// Execute USB command and return status to terminal
			usb_putc(exec_usb_cmd(usart_instance, can_module, cmd_buf, &CAN_flags, &CAN_init_val, cantask_id),
						 cantask_id);

			// flush command buffer
			for (buf_ind = 0; buf_ind < CMD_BUFFER_LENGTH; buf_ind++) {
				cmd_buf[buf_ind] = 0x00;
			}
		}
		usb_send(usart_instance, cantask_id);
		flush_clog();

		// TODO zero task delay used for rescheduling, to prevent interference between
		// simultaneous uart sending of both tasks
		vTaskDelay(0);
	}
}

#pragma clang diagnostic pop



/**
 * handle commands incoming via UART
 *
 * @param usart_instance
 * @param can_instance
 * @param cmd_buf
 * @param CAN_flags
 * @param CAN_init_val
 * @param cantask_id
 * @return
 */
uint8_t exec_usb_cmd(usart_module_t *usart_instance, struct can_module *can_instance, uint8_t *cmd_buf, uint16_t *CAN_flags, struct CAN_init_val_struct *CAN_init_val, uint8_t cantask_id) {

	ulog_s("EXEC USB CMD\r\n");

	struct CAN_tx_msg_struct CAN_tx_msg;	// CAN msg to send

	uint8_t cmd_len = (uint8_t) strlen((char *) cmd_buf);    // get command length
	uint8_t *cmd_buf_pntr = &(*cmd_buf);    // point to start of received string

	cmd_buf_pntr++;        // skip command identifier

	// check if all chars are valid hex chars
	while (*cmd_buf_pntr) {
		if(*cmd_buf_pntr == CR){
			*cmd_buf_pntr = 68;
		}

		if (!isxdigit(*cmd_buf_pntr)) {
			ulog_s("invalid byte received via UART");
			xlog(cmd_buf_pntr,1);
			return ERROR;
		}
		++cmd_buf_pntr;
	}
	cmd_buf_pntr = &(*cmd_buf);    // reset pointer

	uint8_t *tmp_pntr;        // temporary pointer for ACR/AMR
	uint8_t tmp_regdata;    // temporary used for register data

	switch (*cmd_buf_pntr) {
		// get serial number
		case GET_SERIAL:
			ulog_s("request serial");
			usb_putc(GET_SERIAL,cantask_id);
			usb_puts((uint8_t *) SERIAL,cantask_id);
			return CR;

			// get hard- and software version
		case GET_VERSION:
			ulog_s("request version");
			usb_putc(GET_VERSION,cantask_id);
			usb_byte2ascii(HW_VER,cantask_id);
			usb_byte2ascii(SW_VER,cantask_id);
			return CR;

			// get only software version
		case GET_SW_VERSION:
			ulog_s("request sw version");
			usb_putc(GET_SW_VERSION,cantask_id);
			usb_byte2ascii(SW_VER_MAJOR,cantask_id);
			usb_byte2ascii(SW_VER_MINOR,cantask_id);
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
			if (!checkbit(CAN_flags, BUS_ON))
				return ERROR;

			usb_putc(READ_STATUS,cantask_id);
			usb_byte2ascii((uint8_t) (*CAN_flags >> 8),cantask_id);

			// turn off Bus Error indication
			port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);
			ulog_s("bus error indication turned off.\r\n");

			// reset error flags
			reset_can_errorflags(CAN_flags);
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
			if (checkbit(CAN_flags, BUS_ON))
				return ERROR;

			// assign pointer to AMR or ACR values depending on command
			if (*cmd_buf_pntr == SET_AMR)
				tmp_pntr = CAN_init_val->amr;
			else
				tmp_pntr = CAN_init_val->acr;

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
			return reset_can_errorflags(CAN_flags);

			// set bitrate via BTR
		case SET_BTR:
			// set fix bitrate
		case SET_BITRATE:
			port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);
			ulog_s("set bitrate");
			if ((cmd_len != 5) && (cmd_len != 2)) {
				ulog_s("invalid cmd length");
				return ERROR;    // check valid cmd length
			}
			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
				ulog_s("controller is in reset");
				return ERROR;
			}

			// store user or fixed bit rate
			if (*cmd_buf_pntr == SET_BTR) {
				CAN_init_val->btr0 = ascii2byte(++cmd_buf_pntr);
				CAN_init_val->btr0 <<= 4;
				CAN_init_val->btr0 |= ascii2byte(++cmd_buf_pntr);
				CAN_init_val->btr1 = ascii2byte(++cmd_buf_pntr);
				CAN_init_val->btr1 <<= 4;
				CAN_init_val->btr1 |= ascii2byte(++cmd_buf_pntr);
				CAN_init_val->fixed_rate = 0;
			} else {
				CAN_init_val->fixed_rate = *(++cmd_buf_pntr);
			}
			// init CAN controller
			setbit(CAN_flags, CAN_INIT);    // indicate initialized controller
			return reset_can_errorflags(CAN_flags);

			// open CAN channel
		case OPEN_CAN_CHAN:
			ulog_s("open CAN channel");
			// return error if controller is not initialized or already open
			if (!checkbit(CAN_flags, CAN_INIT)) {
				ulog_s("controller not initialized or already open");
				return ERROR;
			}

			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
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
			setbit(CAN_flags, BUS_ON);
			return CR;

			// close CAN channel
		case CLOSE_CAN_CHAN:
			ulog_s("close CAN channel");
			// check if CAN controller is in reset mode
			if (!checkbit(CAN_flags, BUS_ON))
				return ERROR;

			// switch to reset mode
			do {        // do as long as RM_RR_Bit is not set
#if defined(ENABLE_SELFTEST)
				ModeControlReg = _BV (RM_RR_Bit) | _BV (STM_Bit);
#else
				ModeControlReg = _BV(RM_RR_Bit);
#endif
			} while ((ModeControlReg & _BV(RM_RR_Bit)) != _BV(RM_RR_Bit));
			clearbit(CAN_flags, BUS_ON);
			return CR;

			// send R11bit ID message
		case SEND_R11BIT_ID:
			ulog_s("send 11bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY)) {
				ulog_s("\r\nCAN controller in reset mode or busy");
				return ERROR;
			}
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

			return transmit_CAN(can_instance, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

		case SEND_11BIT_ID:
			ulog_s("send 11bit ID message");
			// check if CAN controller is in reset mode or busy
			setbit(CAN_flags,BUS_ON);
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY)) {
				xlog((uint8_t *) &CAN_flags, 2);
				ulog_s("\r\nCAN controller in reset mode or busy");
				return ERROR;
			}

			if ((cmd_len < 5) || (cmd_len > 21)) {
				ulog_s("invalid cmd length");
				xlog(&cmd_len,1);
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
			// check number of data bytes supplied against data length byte
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
			// if transmit buffer was empty send message
			return transmit_CAN(can_instance, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

			// send 29bit ID message
		case SEND_R29BIT_ID:
			ulog_s("send R29bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY))
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
			return transmit_CAN(can_instance, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

		case SEND_29BIT_ID:
			ulog_s("send 29bit ID message");
			// check if CAN controller is in reset mode or busy
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY))
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
			return transmit_CAN(can_instance, CAN_RX_STANDARD_FILTER_ID_0, CAN_tx_msg.data);

			// read Error Capture Register
			// read Arbitration Lost Register
		case READ_ECR:
		case READ_ALCR:
			ulog_s("read ECR/ALCR");
			// check if CAN controller is in reset mode
			if (!checkbit(CAN_flags, BUS_ON))
				return ERROR;

			if (*cmd_buf_pntr == READ_ECR) {
				usb_putc(READ_ECR,cantask_id);
				usb_byte2ascii(last_ecr,cantask_id);
			} else {
				usb_putc(READ_ALCR,cantask_id);
				usb_byte2ascii(last_alc,cantask_id);
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
			usb_putc(READ_REG,cantask_id);
			usb_byte2ascii(read_CAN_reg(cmd_len),cantask_id);
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
			if (!checkbit(CAN_flags, CAN_INIT))
				ulog_s("can not initlzd");
			return ERROR;

			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON))
				ulog_s("can ctrl in reset");
			return ERROR;

			// switch to listen only mode
			/*	do {
					ModeControlReg = _BV(LOM_Bit);
				} while ((ModeControlReg & _BV(RM_RR_Bit)) == _BV(RM_RR_Bit));*/
			setbit(CAN_flags, BUS_ON);
			return CR;

			// end with error on unknown commands
		default:
			ulog_s("unknown command\r\n");
			return ERROR;
	}                // end switch

	// we should never reach this return
	return ERROR;
}                // end exec_usb_cmd


/*
void init_can_mem() {
	/* Initialize the memory. *//*
	for (uint32_t i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
		tx_message_0[i] = (uint8_t) i;
		tx_message_1[i] = (uint8_t) (i + 0x80);
	}
}
*/

TaskHandle_t vCreateCanTask(cantask_params *params) {

	ulog_s("creating CAN Task...\r\n");

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vCanTask,       /* Function that implements the task. */
			"canTASK",          /* Text name for the task. */
			250,      /* Stack size in words, not bytes. */
			params,    /* Parameter passed into the task. */
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

