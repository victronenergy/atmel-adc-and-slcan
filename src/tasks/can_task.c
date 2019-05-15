//
// Created by stekreis on 25.05.18.
//

#include <log.h>
#include "can_task.h"
#include "usb.h"
#include <ctype.h>
#include <usb.h>
#include <cmsis/samc21/include/component/can.h>
#include <can.h>

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
uint8_t exec_usb_cmd(struct can_module *can_module, Can *can_instance, uint8_t *cmd_buf, uint16_t *CAN_flags, struct CAN_init_val_struct *CAN_init_val, uint8_t cantask_id, uint32_t *bitrate);
void can_send_standard_message(struct can_module *can_instance, uint32_t id_value, uint8_t *data);
uint8_t reset_can_errorflags(uint16_t *CAN_flags);
uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element);
uint8_t read_CAN_reg(uint8_t reg);
void write_CAN_reg(uint8_t cmd_len, uint8_t tmp_regdata);
void set_can_init_values(struct CAN_init_val_struct *can_init_val_struct, uint16_t CAN_flags);
bool adapt_rx_can_msg(struct can_module *const can_module, struct CAN_rx_msg_struct *CAN_rx_msg);
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate);

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


// can CAN interface
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate){

	/* Initialize the module. */
	struct can_config config_can;
	can_get_config_defaults(&config_can);

	config_can.nonmatching_frames_action_standard = CAN_NONMATCHING_FRAMES_FIFO_0;
	config_can.nonmatching_frames_action_extended = CAN_NONMATCHING_FRAMES_FIFO_1;
	config_can.remote_frames_standard_reject = false;
	config_can.remote_frames_extended_reject = false;
	config_can.extended_id_mask = 0x00000000;

	// make sure that the hw is off.
	can_stop(can_module);

	//setup the hw
	can_init(can_module, can_hw, &config_can);

	// change bitrate
//	can_set_baudrate(can_hw, bitrate);

	// start hw
	can_start(can_module);
}


uint8_t reset_can_errorflags(uint16_t *CAN_flags) {
	port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);
	return CR;
}

uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element) {
	volatile CAN_TXFQS_Type *fifo_status = (CAN_TXFQS_Type *) &(can_module->hw->TXFQS);
	uint32_t  fifo_put_index = fifo_status->bit.TFQPI + 0;
	enum status_code status;
	if (!(fifo_status->bit.TFQF + 0)) {
		status = can_set_tx_buffer_element(can_module, tx_element, fifo_put_index);
		if (status != STATUS_OK) {
			c_log_s("E1");
			return ERROR;
		}
		status = can_tx_transfer_request(can_module, 1u << fifo_put_index);
		if (status != STATUS_OK) {
			c_log_s("E2");
			return ERROR;
		}
		return NO_RETURN;
	} else {
		c_log_s("E3");
		return ERROR;
	}

}

//#pragma clang diagnostic push
//#pragma clang diagnostic ignored "-Wmissing-noreturn"

void set_can_init_values(struct CAN_init_val_struct *can_init_val_struct, uint16_t CAN_flags){
	reset_can_errorflags(&CAN_flags);

	memset(can_init_val_struct->acr, 0x00, 4);
	memset(can_init_val_struct->amr, 0xff, 4);
	can_init_val_struct->btr0 = BTR0_100k;
	can_init_val_struct->btr1 = BTR1_100k;
	can_init_val_struct->fixed_rate = 0x00;
}


bool adapt_rx_can_msg(struct can_module *const can_module, struct CAN_rx_msg_struct *CAN_rx_msg) {
	bool messsage_read = false;
	//FIFO buffers status
	volatile CAN_RXF0S_Type *fifo0_status = (CAN_RXF0S_Type *) &(can_module->hw->RXF0S);
	volatile CAN_RXF1S_Type *fifo1_status = (CAN_RXF1S_Type *) &(can_module->hw->RXF1S);

	//FIFO buffers fill level
	uint32_t fifo0_fill_level = (uint32_t) fifo0_status->bit.F0FL;
	uint32_t fifo1_fill_level = (uint32_t) fifo1_status->bit.F1FL;

	// check for errors
	volatile uint32_t status = can_read_interrupt_status(can_module);
/*	if ((status & CAN_PROTOCOL_ERROR_ARBITRATION)
		|| (status & CAN_PROTOCOL_ERROR_DATA)) {
		port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_ARBITRATION
												| CAN_PROTOCOL_ERROR_DATA);
		//ulog_s("Protocol error, please double check the clock in two boards. \r\n\r\n");
		c_log('p');
	}*/

	if (status & CAN_PROTOCOL_ERROR_ARBITRATION) {
		port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_ARBITRATION);
		//ulog_s("Protocol error, please double check the clock in two boards. \r\n\r\n");
		c_log_s("p1");
	}
	if (status & CAN_PROTOCOL_ERROR_DATA) {
		port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_DATA);
		//ulog_s("Protocol error, please double check the clock in two boards. \r\n\r\n");
		c_log_s("p2");
	}

	// fifo0 is used for standard messages
	if(fifo0_fill_level) {
		uint32_t fifo0_getindex = fifo0_status->bit.F0GI + 0;	//returns the current index of the receive buffer
		struct can_rx_element_fifo_0 rx_element_fifo_0;

//		c_log_s("f0");
		// fifo0 contains (new) data

		if(fifo0_status->bit.RF0L){
			ulog_s("message lost!F0\r\n");
		}
		if(fifo0_status->bit.F0F){
			ulog_s("FIFO full!F0\r\n");
			ulog_s("level: ");
			xlog((uint8_t*) &fifo0_fill_level, 4);
			ulog_s("\r\nindex pos: ");
			xlog((uint8_t*) &fifo0_getindex, 1);
			ulog_s("\r\n");
		}

		if (can_get_rx_fifo_0_element(can_module, &rx_element_fifo_0, fifo0_getindex) == STATUS_OK) {

			CAN_rx_msg->format = STANDARD_FRAME;
			CAN_rx_msg->id = rx_element_fifo_0.R0.bit.ID;
			CAN_rx_msg->rtr = (uint8_t) rx_element_fifo_0.R0.bit.RTR;
			CAN_rx_msg->len = (uint8_t) (rx_element_fifo_0.R1.bit.DLC + 0);
			memcpy(CAN_rx_msg->data, rx_element_fifo_0.data, CAN_rx_msg->len);
			messsage_read = true;
		}
		can_rx_fifo_acknowledge(can_module, 0, fifo0_getindex);
	}
	// fifo1 is used for extended messages
	else if (fifo1_fill_level) {
		uint32_t fifo1_getindex = fifo1_status->bit.F1GI + 0;	//returns the current index of the receive buffer
		struct can_rx_element_fifo_1 rx_element_fifo_1;
//		c_log_s("f1");
		// fifo1 contains (new) data

		if(fifo1_status->bit.RF1L){
			ulog_s("message lost!F1\r\n");
		}
		if(fifo1_status->bit.F1F){
			ulog_s("FIFO full!F1\r\n");
		}

		if (can_get_rx_fifo_1_element(can_module, &rx_element_fifo_1, fifo1_getindex) == STATUS_OK) {

			CAN_rx_msg->format = EXTENDED_FRAME;
			CAN_rx_msg->id = rx_element_fifo_1.R0.bit.ID;
			CAN_rx_msg->rtr = (uint8_t) rx_element_fifo_1.R0.bit.RTR;
			CAN_rx_msg->len = (uint8_t) (rx_element_fifo_1.R1.bit.DLC + 0);
			memcpy(CAN_rx_msg->data, rx_element_fifo_1.data, CAN_rx_msg->len);

			messsage_read = true;
		}
		can_rx_fifo_acknowledge(can_module, 1, fifo1_getindex);
	}
	return messsage_read;
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
	Can *can_instance = params->can_instance;

	usart_module_t *usart_instance = params->usart_instance;
	uint8_t cantask_id = params->task_id;

	// can structure to control the can hw instance
	struct can_module can_module;
	uint32_t can_bitrate = 0;

	// uart data struct for rx/tx buffers (allows multi buffering)
	usart_buf_t usart_buf;


	//initially both LEDs off
	port_pin_set_output_level(LEDPIN_C21_GREEN, LED_INACTIVE);
	port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);

	configure_usart_callbacks(usart_instance, cantask_id);

	struct CAN_rx_msg_struct CAN_rx_msg;

	//initialize CAN error status
	struct CAN_init_val_struct CAN_init_val;
	set_can_init_values(&CAN_init_val, CAN_flags);

	start_canbus_usart(usart_instance, &usart_buf, cantask_id);

	ulog_s("start CAN Task ");
	uint8_t tmp = (uint8_t) (cantask_id + 0x30);
	ulog( &tmp,1);
	ulog_s("...\r\n");

	for (;;) {


//		if (cantask_id)
			port_pin_set_output_level(PIN_PA15, true);
//		else
//			port_pin_set_output_level(PIN_PA24, true);

		/**
		 * UART TO CAN HANDLING
		 */
		uint32_t buf_num = 0;
		uint8_t *buf = NULL;
		enum_usb_return_t new_cmd_available = get_complete_cmd(&buf, &buf_num, cantask_id);
		if (new_cmd_available == NEW_CMD || new_cmd_available == BUF_OVERRUN) {
//			ulog_s(" new: ");
//			xlog((uint8_t *) &buf_num,1);
//			ulog_s(" ");
//			xlog(buf, 10);
			// Execute USB command and return status to terminal
			uint8_t r = exec_usb_cmd(&can_module, can_instance, buf, &CAN_flags, &CAN_init_val, cantask_id, &can_bitrate);
//			xlog((uint8_t *) &r,1);
			if (r != NO_RETURN) { // check if we have to send something back to the host.
				usb_putc(r, cantask_id);
				usb_send(usart_instance, cantask_id);
			}
			// flush command buffer
			clear_cmd_buf(cantask_id, buf_num);
		}

//		if (cantask_id) {
			port_pin_set_output_level(PIN_PA15, false);
			port_pin_set_output_level(PIN_PA14, true);
//		} else {
//			port_pin_set_output_level(PIN_PA24, false);
//			port_pin_set_output_level(PIN_PA25, true);
//		}


		/**
		 * CAN TO UART HANDLING
		 */
		if (checkbit(&CAN_flags, BUS_ON)) {
			bool new_can_message = adapt_rx_can_msg(&can_module, &CAN_rx_msg);
			if (new_can_message) {
//			c_log('n');
				// check frame format
				if (CAN_rx_msg.format == STANDARD_FRAME) {    // Standard Frame
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
				usb_send(usart_instance, cantask_id);
			}
		}
//		if (cantask_id)
			port_pin_set_output_level(PIN_PA14, false);
//		else
//			port_pin_set_output_level(PIN_PA25, false);
		flush_clog();

		// TODO zero task delay used for rescheduling, to prevent interference between
		// simultaneous uart sending of both tasks
		vTaskDelay(0);
	}
}

//#pragma clang diagnostic pop



/**
 * handle commands incoming via UART
 *
 * @param can_instance
 * @param cmd_buf
 * @param CAN_flags
 * @param CAN_init_val
 * @param cantask_id
 * @return
 */
uint8_t exec_usb_cmd(struct can_module *can_module, Can * can_instance, uint8_t *cmd_buf, uint16_t *CAN_flags, struct CAN_init_val_struct *CAN_init_val, uint8_t cantask_id, uint32_t *can_bitrate) {

	struct CAN_tx_msg_struct CAN_tx_msg;	// CAN msg to send
	struct can_tx_element tx_element;

	uint8_t cmd_len = (uint8_t) strlen((char *) cmd_buf);    // get command length
	uint8_t *cmd_buf_pntr = cmd_buf;    // point to start of received string

	cmd_buf_pntr++;        // skip command identifier


	// check if all chars are valid hex chars
	//TODO possible endless loop!!
	while (*cmd_buf_pntr) {
		if(*cmd_buf_pntr == CR){
			*cmd_buf_pntr = 68;
		}

		if (!isxdigit(*cmd_buf_pntr)) {
			c_log('x');
			return ERROR;
		}
		++cmd_buf_pntr;
	}
	cmd_buf_pntr = cmd_buf;    // reset pointer

	uint8_t *tmp_pntr;        // temporary pointer for ACR/AMR

//	ulog_s("\r\nBUF: ");
//	xlog(cmd_buf, cmd_len);

//	c_log('|');

	switch (*cmd_buf_pntr) {
		// get serial number
		case GET_SERIAL:
			c_log('N');
			usb_putc(GET_SERIAL,cantask_id);
			usb_puts((uint8_t *) SERIAL,cantask_id);
			return CR;

			// get hard- and software version
		case GET_VERSION:
//			c_log('V');
			usb_putc(GET_VERSION,cantask_id);
			usb_byte2ascii(HW_VER,cantask_id);
			usb_byte2ascii(SW_VER,cantask_id);
			return CR;

			// get only software version
		case GET_SW_VERSION:
			c_log('v');
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
			c_log('F');

			uint8_t flags = 0;
			uint32_t status = can_read_protocal_status(can_module);
			flags |= (status & CAN_PSR_EW) ? (1 << 2) : 0;
			flags |= (status & CAN_PSR_EP) ? (1 << 5) : 0;
			flags |= (can_module->hw->RXF0S.bit.RF0L || can_module->hw->RXF1S.bit.RF1L) ? (1 << 3) : 0;
//			flags |= (status & CAN_PSR_LEC_Msk) ? (1 << 7) : 0;
			if ((status & CAN_PSR_LEC_Msk) && (status & CAN_PSR_LEC_Msk) != 0x7) {
				flags |= (1 << 7);
			}

//			flags = (status & CAN_PSR_LEC_Msk);

//			c_log('f');
//			c_log(flags);


			usb_putc(READ_STATUS,cantask_id);
			usb_byte2ascii((uint8_t) flags,cantask_id);

			// reset error flags
			reset_can_errorflags(CAN_flags);
			return CR;

			// set AMR
		case SET_AMR:
			c_log('m');
			// set ACR
		case SET_ACR:
			c_log('M');
			// check valid cmd length and if CAN was initialized before
			if (cmd_len != 9) {
				c_log('e');
				return ERROR;
			}
			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
				c_log('e');
				return ERROR;
			}

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

			// set fix bitrate
		case SET_BITRATE:
			c_log('S');
			port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);

			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
				c_log('e');
				return ERROR;
			}

//			ulog_s("\r\ncmd_len: ");
//			xlog(&cmd_len, 1);
			if ((cmd_len != 5) && (cmd_len != 2)) {
				c_log('e');
				return ERROR;    // check valid cmd length
			}
			uint8_t value = (uint8_t) (*(++cmd_buf_pntr) - 0x30);
			// check if value is in bound of array


			c_log(value+0x30u);

//			value = 5;
			if (value > 8) {
				c_log('e');
				return ERROR;
			}

			const uint32_t fixed_rate[] = {10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000};

			*can_bitrate = fixed_rate[value];
			setbit(CAN_flags, CAN_INIT); // indicate initialized controller

			return CR;

			// open CAN channel
		case OPEN_CAN_CHAN:
			c_log('O');
			// return error if controller is not initialized
			if (!checkbit(CAN_flags, CAN_INIT)) {
				c_log_s("e1");
				return ERROR;
			}

			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
				c_log_s("e2");
				return ERROR;
			}

//			can_start(can_module);
			setup_can_instance(can_module, can_instance, *can_bitrate);
			setbit(CAN_flags, BUS_ON);
			return CR;

			// close CAN channel
		case CLOSE_CAN_CHAN:
			c_log('C');
			if (!checkbit(CAN_flags, BUS_ON)) {
				c_log('e');
				return ERROR;
			}
			can_stop(can_module);
			clearbit(CAN_flags, BUS_ON);
			clearbit(CAN_flags, CAN_INIT);
			return CR;

			// send R11bit ID message
		case SEND_R11BIT_ID:
			c_log('r');

			// check if CAN controller is in reset mode or busy
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY)) {
//				c_log('e');
				return ERROR;
			}
			// check valid cmd length (only 5 bytes for RTR)
			if (cmd_len != 5) {
//				c_log('e');
				return ERROR;
			}

			can_get_tx_buffer_element_defaults(&tx_element);
			CAN_tx_msg.rtr = 1;    // remote transmission request

			// store std. frame format
			CAN_tx_msg.format = 0;
			// store ID
			CAN_tx_msg.id = ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);

			tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(CAN_tx_msg.id) | CAN_TX_ELEMENT_T0_RTR;

			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			tx_element.T1.bit.DLC = CAN_tx_msg.len;

			// if transmit buffer was empty send message
			return transmit_CAN(can_module, &tx_element);

		case SEND_11BIT_ID:
			c_log('t');
			// check if CAN controller is in reset mode or busy
			setbit(CAN_flags,BUS_ON);
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY)) {
				c_log_s("e1");
				return ERROR;
			}


			if ((cmd_len < 5) || (cmd_len > 21)) {
//				c_log_s("e2");
//				xlog(&cmd_len,1);
				return ERROR;    // check valid cmd length
			}

			can_get_tx_buffer_element_defaults(&tx_element);
			CAN_tx_msg.rtr = 0;    // no remote transmission request

			// store std. frame format
			CAN_tx_msg.format = 0;
			// store ID
			CAN_tx_msg.id = ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);
			CAN_tx_msg.id <<= 4;
			CAN_tx_msg.id += ascii2byte(++cmd_buf_pntr);

			tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(CAN_tx_msg.id);

			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			// check number of data bytes supplied against data length byte
//			ulog_s("\r\ncmd_len: ");
//			xlog(&cmd_len, 1);
//			ulog_s(" msg_len: ");
//			xlog(&(CAN_tx_msg.len), 1);

			if (CAN_tx_msg.len != ((cmd_len - 5) / 2)) {
//				c_log_s("e3");
//				c_log(((cmd_len - 5) / 2)+ 0x30);
				return ERROR;
			}
//			c_log(((cmd_len - 5) / 2)+ 0x30);

			// check for valid length
			if (CAN_tx_msg.len > 8) {
//				c_log_s("e4");
				return ERROR;
			} else {        // store data
				tx_element.T1.bit.DLC = CAN_tx_msg.len;
				// cmd_len is no longer needed, so we can use it as counter here
				for (cmd_len = 0; cmd_len < CAN_tx_msg.len; cmd_len++) {
					cmd_buf_pntr++;
//					CAN_tx_msg.data[cmd_len] = ascii2byte(cmd_buf_pntr);
//					CAN_tx_msg.data[cmd_len] <<= 4;
					tx_element.data[cmd_len] = ascii2byte(cmd_buf_pntr);
					tx_element.data[cmd_len] <<=  4;
					cmd_buf_pntr++;
//					CAN_tx_msg.data[cmd_len] += ascii2byte(cmd_buf_pntr);
					tx_element.data[cmd_len] += ascii2byte(cmd_buf_pntr);
				}
			}
			// if transmit buffer was empty send message
			return transmit_CAN(can_module, &tx_element);

			// send 29bit ID message
		case SEND_R29BIT_ID:
			c_log('R');
			// check if CAN controller is in reset mode or busy
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY)) {
//				c_log('e');
				return ERROR;
			}

			if (cmd_len != 10) {
//				c_log('e');
				return ERROR;    // check valid cmd length
			}

			can_get_tx_buffer_element_defaults(&tx_element);

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

			tx_element.T0.reg |= CAN_TX_ELEMENT_T0_EXTENDED_ID(CAN_tx_msg.id) | CAN_TX_ELEMENT_T0_XTD | CAN_TX_ELEMENT_T0_RTR;

			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			tx_element.T1.bit.DLC = CAN_tx_msg.len;
			// if transmit buffer was empty send message
			return transmit_CAN(can_module, &tx_element);

		case SEND_29BIT_ID:
//			c_log('T');
			// check if CAN controller is in reset mode or busy
			if (!checkbit(CAN_flags, BUS_ON) || checkbit(CAN_flags, TX_BUSY)) {
				c_log_s("e1");
				return ERROR;
			}

			if ((cmd_len < 10) || (cmd_len > 26)) {
				c_log_s("e2");
				return ERROR;    // check valid cmd length
			}
			can_get_tx_buffer_element_defaults(&tx_element);


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

			tx_element.T0.reg |= CAN_TX_ELEMENT_T0_EXTENDED_ID(CAN_tx_msg.id) | CAN_TX_ELEMENT_T0_XTD;

			// store data length
			CAN_tx_msg.len = ascii2byte(++cmd_buf_pntr);
			// check number of data bytes supplied against data lenght byte
			if (CAN_tx_msg.len != ((cmd_len - 10) / 2)) {
				c_log('T');
				c_log_s("e3#");
				xlog(&cmd_len,1);

//				c_log(((cmd_len - 10) / 2) + 0x30);
				return ERROR;
			}
//			c_log('#');
//			c_log(((cmd_len - 10) / 2) + 0x30);

			// check for valid length
			if (CAN_tx_msg.len > 8) {
				c_log_s("e4");
				return ERROR;
			} else {        // store data
				tx_element.T1.bit.DLC = CAN_tx_msg.len;
				// cmd_len is no longer needed, so we can use it as counter here
				for (cmd_len = 0; cmd_len < CAN_tx_msg.len; cmd_len++) {
					cmd_buf_pntr++;
//					CAN_tx_msg.data[cmd_len] = ascii2byte(cmd_buf_pntr);
//					CAN_tx_msg.data[cmd_len] <<= 4;
					tx_element.data[cmd_len] = ascii2byte(cmd_buf_pntr);
					tx_element.data[cmd_len] <<=  4;
					cmd_buf_pntr++;
//					CAN_tx_msg.data[cmd_len] += ascii2byte(cmd_buf_pntr);
					tx_element.data[cmd_len] += ascii2byte(cmd_buf_pntr);
				}
			}
			// if transmit buffer was empty send message
			return transmit_CAN(can_module, &tx_element);

			// read Error Capture Register
			// read Arbitration Lost Register
		case READ_ECR:
			c_log('E');
		case READ_ALCR:
			c_log('A');
//			ulog_s("read ECR/ALCR");
			// check if CAN controller is in reset mode
			if (!checkbit(CAN_flags, BUS_ON)) {
				c_log('e');
				return ERROR;
			}

			if (*cmd_buf_pntr == READ_ECR) {
				usb_putc(READ_ECR,cantask_id);
				usb_byte2ascii(last_ecr,cantask_id);
			} else {
				usb_putc(READ_ALCR,cantask_id);
				usb_byte2ascii(last_alc,cantask_id);
			}
			return CR;

		case LISTEN_ONLY:
			c_log('L');
//			ulog_s("listen only mode");
			// return error if controller is not initialized or already open
			if (!checkbit(CAN_flags, CAN_INIT)) {
				c_log('e');
				return ERROR;
			}

			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
				c_log('e');
				return ERROR;
			}

			// switch to listen only mode
			/*	do {
					ModeControlReg = _BV(LOM_Bit);
				} while ((ModeControlReg & _BV(RM_RR_Bit)) == _BV(RM_RR_Bit));*/
			setup_can_instance(can_module, can_instance, *can_bitrate);
			setbit(CAN_flags, BUS_ON);
			return CR;

			// end with error on unknown commands
		default:
			c_log('u');
			xlog(cmd_buf_pntr,1);
			return ERROR;
	}                // end switch

	// we should never reach this return
	c_log('X');
	return ERROR;
}                // end exec_usb_cmd


/*
void init_can_mem() {
	// Initialize the memory.
	for (uint32_t i = 0; i < CONF_CAN_ELEMENT_DATA_SIZE; i++) {
		tx_message_0[i] = (uint8_t) i;
		tx_message_1[i] = (uint8_t) (i + 0x80);
	}
}
*/

TaskHandle_t vCreateCanTask(cantask_params *params) {

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	char task_name[8] = "cTASK_";
	task_name[6] = (uint8_t) (params->task_id+0x30);
	task_name[7] = '\0';

	ulog_s("creating CAN Task ");
	ulog((uint8_t*) &task_name[6],1);
	ulog_s("...\r\n");

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vCanTask,       /* Function that implements the task. */
			task_name,          /* Text name for the task. */
			350,      /* Stack size in words, not bytes. */
			params,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY + 5,/* Priority at which the task is created. */
			&xHandle);      /* Used to pass out the created task's handle. */

	if (xReturned == pdPASS) {
		ulog_s("successfully created CAN Task ");
		ulog((uint8_t*) &task_name[6],1);
		ulog_s("\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		//vTaskDelete( xHandle );
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;


}

