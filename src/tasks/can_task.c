//
// Created by stekreis on 25.05.18.
//

#include <log.h>
#include "can_task.h"
#include <ctype.h>
#include <usb.h>
#include <cmsis/samc21/include/component/can.h>
#include <can.h>
#include "slcan.h"
#include "uart_commands.h"

// TODO END

#define STANDARD_FRAME 0
#define EXTENDED_FRAME 1

// CAN rx message
struct CAN_rx_msg_struct{
	uint8_t format;        // Extended/Standard Frame
	uint32_t id;            // Frame ID
	uint8_t rtr;            // RTR/Data Frame
	uint8_t len;            // Data Length
	uint8_t data[8];        // Data Bytes
};                  // length 15 byte/each

/*
 * Prototypes
 */
void vCanTask(void *pvParameters);
uart_command_return_t exec_uart_cmd(struct can_module *can_module, Can *can_instance, uint8_t *cmd_buf, can_flags_t *can_flags, uint8_t cantask_id, uint32_t *can_bitrate);
bool adapt_rx_can_msg(struct can_module *const can_module, struct CAN_rx_msg_struct *CAN_rx_msg);
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate);


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
	can_flags_t can_flags = {.bus_on = false, .init_complete = false, .tx_busy = false};

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

	start_canbus_usart(usart_instance, &usart_buf, cantask_id);

	ulog_s("start CAN Task ");
	uint8_t tmp = (uint8_t) (cantask_id + 0x30);
	ulog( &tmp,1);
	ulog_s("...\r\n");

	for (;;) {


//		if (cantask_id)
//			port_pin_set_output_level(PIN_PA15, true);
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
			uint8_t r = exec_uart_cmd(&can_module, can_instance, buf, &can_flags, cantask_id, &can_bitrate);
//			xlog((uint8_t *) &r,1);
			if (r != NO_RETURN && r != ERROR_BUSY) { // check if we have to send something back to the host.
				usb_putc(r, cantask_id);
				usb_send(usart_instance, cantask_id);
			}
			// flush command buffer
			if (r != ERROR_BUSY) {
				clear_cmd_buf(usart_instance, cantask_id, buf_num);
			}
		}

//		if (cantask_id) {
//			port_pin_set_output_level(PIN_PA15, false);
//			port_pin_set_output_level(PIN_PA14, true);
//		} else {
//			port_pin_set_output_level(PIN_PA24, false);
//			port_pin_set_output_level(PIN_PA25, true);
//		}


		/**
		 * CAN TO UART HANDLING
		 */
		if (can_flags.bus_on) {
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
				usb_putc(RETURN_CR,cantask_id);
				usb_send(usart_instance, cantask_id);
			}
		}
//		if (cantask_id)
//			port_pin_set_output_level(PIN_PA14, false);
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
 * @param can_flags
 * @param CAN_init_val
 * @param cantask_id
 * @return
 */
uart_command_return_t exec_uart_cmd(struct can_module *can_module, Can *can_instance, uint8_t *cmd_buf, can_flags_t *can_flags, uint8_t cantask_id, uint32_t *can_bitrate) {
	uart_command_return_t return_code = NO_RETURN;

	uint8_t cmd_len = (uint8_t) strlen((char *) cmd_buf);    // get command length
	uint8_t *cmd_buf_pntr = cmd_buf;    // point to start of received string

	cmd_buf_pntr++;        // skip command identifier


	// check if all chars are valid hex chars
	//TODO possible endless loop!!
	while (*cmd_buf_pntr) {
		if(*cmd_buf_pntr == RETURN_CR){
			*cmd_buf_pntr = 68;
		}

		if (!isxdigit(*cmd_buf_pntr)) {
			c_log('x');
			return RETURN_ERROR;
		}
		++cmd_buf_pntr;
	}
	cmd_buf_pntr = cmd_buf;    // reset pointer

//	ulog_s("\r\nBUF: ");
//	xlog(cmd_buf, cmd_len);

//	c_log('|');

	switch (*cmd_buf_pntr) {
		case GET_SERIAL:
			// get serial number
			c_log('N');
			return_code = uart_command_get_serial(cantask_id);
			break;

		case GET_VERSION:
			// get hard- and software version
//			c_log('V');
			return_code = uart_command_get_version(cantask_id);
			break;

		case GET_SW_VERSION:
			// get only software version
			c_log('v');
			return_code = uart_command_get_sw_version(cantask_id);
			break;

		case READ_STATUS:
			// read status flag
			c_log('F');
			return_code = uart_command_read_status(can_module, cantask_id, can_flags);
			break;

		case SET_BITRATE:
			// set fix bitrate
			c_log('S');
			return_code = uart_command_set_bitrate(cmd_len, cmd_buf_pntr, can_bitrate, can_flags);
			break;

		case OPEN_CAN_CHAN:
			// open CAN channel
			c_log('O');
			return_code = uart_command_open_can_channel(can_module, can_instance, can_bitrate, can_flags);
			break;

		case CLOSE_CAN_CHAN:
			// close CAN channel
			c_log('C');
			return_code = uart_command_close_can_channel(can_module, can_flags);
			break;

		case LISTEN_ONLY:
			c_log('L');
			return_code = uart_command_listen_only_mode(can_module, can_instance, can_bitrate, can_flags);
			break;

		case SEND_R11BIT_ID:
			// send R11bit ID message
//			c_log('r');
			return_code = uart_command_send_r11bit_id(can_module, cmd_len, cmd_buf_pntr, can_flags);
			break;

		case SEND_11BIT_ID:
			// send 11bit ID message
//			c_log('t');
			return_code = uart_command_send_11bit_id(can_module, cmd_len, cmd_buf_pntr, can_flags);
			break;

		case SEND_R29BIT_ID:
			// send R29bit ID message
//			c_log('R');
			return_code = uart_command_send_r29bit_id(can_module, cmd_len, cmd_buf_pntr, can_flags);
			break;

		case SEND_29BIT_ID:
			// send 29bit ID message
//			c_log('T');
			return_code = uart_command_send_29bit_id(can_module, cmd_len, cmd_buf_pntr, can_flags);
			break;

/*		case SET_AMR:
			// set AMR
			c_log('m');
			//fallthrough to SET_ACR

		case SET_ACR:
			// set ACR
			c_log('M');
			// check valid cmd length and if CAN was initialized before
			if (cmd_len != 9) {
				c_log('e');
				return RETURN_ERROR;
			}
			// check if CAN controller is in reset mode
			if (checkbit(CAN_flags, BUS_ON)) {
				c_log('e');
				return RETURN_ERROR;
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

		case READ_ECR:
			// read Error Capture Register
			c_log('E');

		case READ_ALCR:
			// read Arbitration Lost Register
			c_log('A');
//			ulog_s("read ECR/ALCR");
			// check if CAN controller is in reset mode
			if (!can_flags->bus_on)) {
				c_log('e');
				return RETURN_ERROR;
			}

			if (*cmd_buf_pntr == READ_ECR) {
				usb_putc(READ_ECR,cantask_id);
				usb_byte2ascii(last_ecr,cantask_id);
			} else {
				usb_putc(READ_ALCR,cantask_id);
				usb_byte2ascii(last_alc,cantask_id);
			}
			return RETURN_CR;
*/

		default:
			// end with error on unknown commands
			c_log('u');
			xlog(cmd_buf_pntr,1);
			return RETURN_ERROR;
	}
	return return_code;
}

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
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
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

