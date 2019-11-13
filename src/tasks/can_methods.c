//
// Created by Felix Hübner on 2019-11-01.
//

#include "can.h"
#include "can_task.h"
#include <usb.h>
#include <control_leds.h>
#include "can_methods.h"
#include "slcan.h"
#include "uart_methods.h"


/******** Methods ********/
/**
 * Read message from CAN and send as slcan message via uart
 * @param can_module can module we want to read from
 * @param cantask_id id of the cantask
 * @param sequence_counter pointer to the sequence_counter (DEBUG)
 * @return return true if a new message was transferred from CAN to UART
 */
bool check_and_transfer_can_message_to_uart(struct can_module *const can_module, uint8_t cantask_id, uint8_t *sequence_counter) {
	bool messsage_read = false;

	// check for errors
	uint32_t status = can_read_interrupt_status(can_module);
	if (status & CAN_PROTOCOL_ERROR_ARBITRATION) {
		set_led(RED_LED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_ARBITRATION);
/*		c_log_s("p1");
		uint32_t prot_status = can_read_protocal_status(can_module);
		if ((prot_status&0x07) != 0) {
			c_log(':');
			c_log((uint8_t) (0x30 + (prot_status & 0x07)));
		}*/
	}
	if (status & CAN_PROTOCOL_ERROR_DATA) {
		set_led(RED_LED, LED_ACTIVE);
		can_clear_interrupt_status(can_module, CAN_PROTOCOL_ERROR_DATA);
/*		c_log_s("p2");
		uint32_t prot_status = can_read_protocal_status(can_module);
		if ((prot_status&0x07) != 0) {
			c_log(':');
			c_log((uint8_t) (0x30 + (prot_status & 0x07)));
		}*/
	}

	//FIFO buffers status
	uint32_t rx_fifo_status = can_rx_get_fifo_status(can_module, 0);

	//FIFO buffers fill level
	uint8_t rx_fifo_fill_level = (uint8_t) (rx_fifo_status & CAN_RXF0S_F0FL_Msk);

	// read element from fifo0 (standard and extended frames)
	if(rx_fifo_fill_level) {
		uint8_t fifo_getindex = (uint8_t) ((rx_fifo_status & CAN_RXF0S_F0GI_Msk) >> CAN_RXF0S_F0GI_Pos); //returns the current index of the receive buffer
		struct can_rx_element_fifo_0 rx_message;

		// lost a message flag is set
		if(rx_fifo_status & CAN_RXF0S_RF0L){
			ulog_s("message lost!F0\r\n");
			set_led(RED_LED, LED_ACTIVE);
		}

		// fifo full flag is set
		if(rx_fifo_status & CAN_RXF0S_F0F){
			ulog_s("FIFO full!F0\r\n");
			set_led(RED_LED, LED_ACTIVE);
		}

		if (can_get_rx_fifo_0_element(can_module, &rx_message, fifo_getindex) == STATUS_OK) {
			uint32_t can_id = 0;
			if (rx_message.R0.bit.XTD) {
				if (!rx_message.R0.bit.RTR) {
					usb_putc(SEND_29BIT_ID, cantask_id);	// can message with 29bit ID
				} else {
					usb_putc(SEND_R29BIT_ID, cantask_id);	// remote message with 29bit ID
				}

				// send ID bytes
				can_id = (uint32_t) (rx_message.R0.bit.ID & CAN_RX_ELEMENT_R0_ID_Msk);
				usb_byte2ascii((uint8_t) ((can_id >> 24) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) ((can_id >> 16) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) ((can_id >> 8) & 0xFF), cantask_id);
				usb_byte2ascii((uint8_t) (can_id & 0xFF), cantask_id);

			} else {
				if (!rx_message.R0.bit.RTR) {
					usb_putc(SEND_11BIT_ID, cantask_id);	// can message with 11bit ID
				} else {
					usb_putc(SEND_R11BIT_ID, cantask_id);	// remote message with 11bit ID
				}

				can_id = rx_message.R0.bit.ID & 0x7FF;

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
			usb_putc((uint8_t) ((rx_message.R1.bit.DLC + 0) + '0'), cantask_id);
			if (!rx_message.R0.bit.RTR) {	// send data only if no remote frame request
				// send data bytes
				for (uint8_t can_rxmsg_pos = 0; can_rxmsg_pos < (rx_message.R1.bit.DLC + 0); can_rxmsg_pos++)
					usb_byte2ascii(rx_message.data[can_rxmsg_pos],cantask_id);
			}
			// send end tag
			usb_putc(RETURN_CR, cantask_id);

			//DEBUG can-sequence checker!
			if (can_id == 4) {
				if (rx_message.data[0] != *sequence_counter) {
					// we have a missmatch!
					*sequence_counter = rx_message.data[0];
				}
				// setup counter for next expected sequence number
				(*sequence_counter)++;
			}

			//acknowledge the message in the can fifo
			can_rx_fifo_acknowledge(can_module, 0, fifo_getindex);
			messsage_read = true;
		}

	}
	return messsage_read;
}


/**
 * method usually resets the errorflags in the SJA1000, here we to not more than reset the ERROR LED
 * @param CAN_flags pointer to the error flags, currently not used
 */
void reset_can_errorflags(can_flags_t *CAN_flags) {
	//currently just the led
	set_led(RED_LED, LED_INACTIVE);
}


/**
 * setup and open the CAN interface
 * @param can_module can module we want to setup and start
 * @param can_hw pointer to the hardware of the can interface
 * @param bitrate bitrate the can hardware should run at
 */
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate){

	/* Initialize the module. */
	struct can_config config_can;
	can_get_config_defaults(&config_can);

	config_can.nonmatching_frames_action_standard = CAN_NONMATCHING_FRAMES_FIFO_0;
	config_can.nonmatching_frames_action_extended = CAN_NONMATCHING_FRAMES_FIFO_0;
	config_can.remote_frames_standard_reject = false;
	config_can.remote_frames_extended_reject = false;
	config_can.extended_id_mask = 0x00000000;
	config_can.run_in_standby = true;
	config_can.automatic_retransmission = false;

	//setup the hw
	can_init(can_module, can_hw, &config_can);

	// change bitrate
	can_set_baudrate(can_hw, bitrate);

	// start hw
	can_start(can_module);
}


/**
 * start sending a can message (transfer a tx_element to the tx_fifo and initiate the sendout)
 * @param can_module can module where we want send the tx_element
 * @param tx_element message to send
 * @return return ERROR if case the copy to the hardware was not successful, ERROR_BUSY if the hardware is still busy sending, or otherwise NO_RETURN
 */
uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element) {
	uint32_t tx_fifo_status = can_tx_get_fifo_queue_status(can_module);
	//extract fifo put index
	uint32_t  tx_fifo_put_index = (uint32_t) ((tx_fifo_status & CAN_TXFQS_TFQPI_Msk) >> CAN_TXFQS_TFQPI_Pos);
	enum status_code status;

#if (CONF_CAN0_TX_FIFO_QUEUE_NUM > 1)
	// check if the fifo is not marked as full
	if (!(tx_fifo_status & CAN_TXFQS_TFQF_Pos)) {
#else
	// check if the index is currently marked as trasmit pending
	if (!(can_tx_get_pending_status(can_module) & (1 << tx_fifo_put_index))) {
#endif
		status = can_set_tx_buffer_element(can_module, tx_element, tx_fifo_put_index);
		if (status != STATUS_OK) {
			return RETURN_ERROR;
		}
		status = can_tx_transfer_request(can_module, 1u << tx_fifo_put_index);
		if (status != STATUS_OK) {
			return RETURN_ERROR;
		}
		return NO_RETURN;
	} else {
		return ERROR_BUSY;
	}

}
