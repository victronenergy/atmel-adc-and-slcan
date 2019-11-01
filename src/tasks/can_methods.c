//
// Created by Felix Hübner on 2019-11-01.
//

#include "can.h"
#include "can_task.h"
#include <log.h>
#include <samc21_slcan_adc.h>
#include <usb.h>
#include "can_methods.h"
#include "slcan.h"
#include "uart_methods.h"


bool check_and_transfer_can_message_to_uart(struct can_module *const can_module, struct can_rx_element_fifo_0 *rx_message, uint8_t cantask_id, uint8_t *sequence_counter) {
	bool messsage_read = false;
	//FIFO buffers status
	uint32_t fifo0_status = can_rx_get_fifo_status(can_module, 0);

	//FIFO buffers fill level
	uint8_t fifo0_fill_level = (uint8_t) (fifo0_status & CAN_RXF0S_F0FL_Msk);

	// check for errors
	volatile uint32_t status = can_read_interrupt_status(can_module);
	if (status & CAN_PROTOCOL_ERROR_ARBITRATION) {
		port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_ARBITRATION);
/*		c_log_s("p1");
		uint32_t prot_status = can_read_protocal_status(can_module);
		if ((prot_status&0x07) != 0) {
			c_log(':');
			c_log((uint8_t) (0x30 + (prot_status & 0x07)));
		}*/
	}
	if (status & CAN_PROTOCOL_ERROR_DATA) {
		port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_DATA);
/*		c_log_s("p2");
		uint32_t prot_status = can_read_protocal_status(can_module);
		if ((prot_status&0x07) != 0) {
			c_log(':');
			c_log((uint8_t) (0x30 + (prot_status & 0x07)));
		}*/
	}

	// read element from fifo0 (standard and extended frames)
	if(fifo0_fill_level) {
		uint8_t fifo0_getindex = (uint8_t) ((fifo0_status & CAN_RXF0S_F0GI_Msk) >> CAN_RXF0S_F0GI_Pos); //returns the current index of the receive buffer

		// lost a message flag is set
		if(fifo0_status & CAN_RXF0S_RF0L){
			ulog_s("message lost!F0\r\n");
			port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		}

		// fifo full flag is set
		if(fifo0_status & CAN_RXF0S_F0F){
			ulog_s("FIFO full!F0\r\n");
			port_pin_set_output_level(LEDPIN_C21_RED, LED_ACTIVE);
		}

		if (can_get_rx_fifo_0_element(can_module, rx_message, fifo0_getindex) == STATUS_OK) {
			uint32_t can_id = 0;
			if (rx_message->R0.bit.XTD) {
				if (!rx_message->R0.bit.RTR) {
					usb_putc(SEND_29BIT_ID, cantask_id);	// can message with 29bit ID
				} else {
					usb_putc(SEND_R29BIT_ID, cantask_id);	// remote message with 29bit ID
				}

				// send ID bytes
				can_id = (uint32_t) (rx_message->R0.bit.ID & CAN_RX_ELEMENT_R0_ID_Msk);
				usb_byte2ascii((uint8_t) ((can_id >> 24) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) ((can_id >> 16) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) ((can_id >> 8) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) (can_id & 0xFF), cantask_id);

			} else {
				if (!rx_message->R0.bit.RTR) {
					usb_putc(SEND_11BIT_ID, cantask_id);	// can message with 11bit ID
				} else {
					usb_putc(SEND_R11BIT_ID, cantask_id);	// remote message with 11bit ID
				}

				can_id = rx_message->R0.bit.ID & 0x7FF;

				//TODO remove hack, needed to get the same address on standard and extended frames! Check if that is expected or not!
				can_id >>= 1;

				// send high byte of ID
				if (((can_id >> 8) & 0x0F) < 10) {
					usb_putc((uint8_t) (((can_id >> 8) & 0x0F) + 48), cantask_id);
				} else {
					usb_putc((uint8_t) (((can_id >> 8) & 0x0F) + 55), cantask_id);
				}
				// send low byte of ID
				usb_byte2ascii((uint8_t) (can_id & 0xFF), cantask_id);
			}

			// send data length code
			usb_putc((uint8_t) ((rx_message->R1.bit.DLC + 0) + '0'), cantask_id);
			if (!rx_message->R0.bit.RTR) {	// send data only if no remote frame request
				// send data bytes
				for (uint8_t can_rxmsg_pos = 0; can_rxmsg_pos < (rx_message->R1.bit.DLC + 0); can_rxmsg_pos++)
					usb_byte2ascii(rx_message->data[can_rxmsg_pos],cantask_id);
			}
			// send end tag
			usb_putc(RETURN_CR, cantask_id);

			//DEBUG can-sequence checker!
			if (can_id == 4) {
				if (rx_message->data[0] != *sequence_counter) {
					// we have a missmatch!
					port_pin_set_output_level(PIN_PA14, true);
					*sequence_counter = rx_message->data[0];
					port_pin_set_output_level(PIN_PA14, false);
				}
				// setup counter for next expected sequence number
				(*sequence_counter)++;
			}

			//acknowledge the message in the can fifo
			can_rx_fifo_acknowledge(can_module, 0, fifo0_getindex);
			messsage_read = true;
		}

	}
	return messsage_read;
}


