//
// Created by Felix Hübner on 2019-05-16.
//

#include <samc21_slcan_adc.h>
#include "can.h"
#include "usb.h"
#include "slcan.h"
#include "uart_commands.h"
#include "can_methods.h"


//TODO move to helper file?

void reset_can_errorflags(can_flags_t *CAN_flags);
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate);
uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element);

void reset_can_errorflags(can_flags_t *CAN_flags) {
	//currently just the led
	port_pin_set_output_level(LEDPIN_C21_RED, LED_INACTIVE);
}

// setup CAN interface
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

// send can message
uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element) {
	volatile CAN_TXFQS_Type *fifo_status = (CAN_TXFQS_Type *) &(can_module->hw->TXFQS);
	uint32_t  fifo_put_index = fifo_status->bit.TFQPI + 0;
	enum status_code status;
	if (!(fifo_status->bit.TFQF + 0)) {
		status = can_set_tx_buffer_element(can_module, tx_element, fifo_put_index);
		if (status != STATUS_OK) {
			return RETURN_ERROR;
		}
		status = can_tx_transfer_request(can_module, 1u << fifo_put_index);
		if (status != STATUS_OK) {
			return RETURN_ERROR;
		}
		return NO_RETURN;
	} else {
		return ERROR_BUSY;
	}

}
//end TODO




uart_command_return_t uart_command_get_serial(uint8_t cantask_id) {
	if (!usb_putc(GET_SERIAL,cantask_id))
		return ERROR_BUSY;
	if (!usb_puts((uint8_t *) SERIAL,cantask_id))
		return ERROR_BUSY;
	return RETURN_CR;
}


uart_command_return_t uart_command_get_version(uint8_t cantask_id) {
	if (!usb_putc(GET_VERSION,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(HW_VER,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(SW_VER,cantask_id))
		return ERROR_BUSY;
	return RETURN_CR;
}


uart_command_return_t uart_command_get_sw_version(uint8_t cantask_id) {
	if (!usb_putc(GET_SW_VERSION,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(SW_VER_MAJOR,cantask_id))
		return ERROR_BUSY;
	if (!usb_byte2ascii(SW_VER_MINOR,cantask_id))
		return ERROR_BUSY;
	return RETURN_CR;
}

uart_command_return_t uart_command_read_status(struct can_module *can_module, uint8_t cantask_id, can_flags_t *can_flags) {
	uint8_t flags = 0;
	// if the bus is not on, the HW is not available and a read from HW resouces will lead to an infinit wait here!
	// so we return 0 if we read before bus-on.
	if (can_flags->bus_on) {
		uint32_t status = can_read_protocal_status(can_module);

		flags |= (status & CAN_PSR_EW) ? (1 << 2) : 0;
		flags |= (status & CAN_PSR_EP) ? (1 << 5) : 0;
		flags |= (can_module->hw->RXF0S.bit.RF0L || can_module->hw->RXF1S.bit.RF1L) ? (1 << 3) : 0;
		if((status & CAN_PSR_LEC_Msk) && (status & CAN_PSR_LEC_Msk) != 0x7) {
			flags |= (1 << 7);
		}
	}

	// reset error flags
	reset_can_errorflags(can_flags);

	if (!usb_putc(READ_STATUS,cantask_id)) {
		return ERROR_BUSY;
	}

	if (!usb_byte2ascii((uint8_t) flags,cantask_id)) {
		return ERROR_BUSY;
	}
	return RETURN_CR;
}


uart_command_return_t uart_command_set_bitrate(uint8_t cmd_len, uint8_t *cmd_buf_pntr, uint32_t *can_bitrate, can_flags_t *can_flags) {
	// check if CAN controller is in reset mode
	if (can_flags->bus_on) {
		return RETURN_ERROR;
	}

	if ((cmd_len != 5) && (cmd_len != 2)) {
		return RETURN_ERROR;    // check valid cmd length
	}
	uint8_t value = (uint8_t) (*(++cmd_buf_pntr) - 0x30);

	// check if value is in bound of array
	if (value > 8) {
		return RETURN_ERROR;
	}

	const uint32_t fixed_rate[] = {10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000};

	*can_bitrate = fixed_rate[value];
	can_flags->init_complete = true; // indicate initialized controller

	return RETURN_CR;
}


uart_command_return_t uart_command_open_can_channel(struct can_module *can_module, Can *can_instance, uint32_t *can_bitrate, can_flags_t *can_flags) {
	// return error if controller is not initialized
	if (!can_flags->init_complete) {
		return RETURN_ERROR;
	}

	// check if CAN controller is in reset mode
	if (can_flags->bus_on) {
		return RETURN_ERROR;
	}

	setup_can_instance(can_module, can_instance, *can_bitrate);
	can_flags->bus_on = true;
	return RETURN_CR;
}


uart_command_return_t uart_command_close_can_channel(struct can_module *can_module, can_flags_t *can_flags) {
	if (!can_flags->bus_on) {
		return RETURN_ERROR;
	}
	can_stop(can_module);
	can_flags->bus_on = false;
	can_flags->init_complete = false;
	return RETURN_CR;
}


uart_command_return_t uart_command_listen_only_mode(struct can_module *can_module, Can *can_instance, uint32_t *can_bitrate, can_flags_t *can_flags) {
	// return error if controller is not initialized or already open
	if (!can_flags->init_complete) {
		return RETURN_ERROR;
	}

	// check if CAN controller is in reset mode
	if (can_flags->bus_on) {
		return RETURN_ERROR;
	}

	// switch to listen only mode
	setup_can_instance(can_module, can_instance, *can_bitrate);
	can_flags->bus_on = true;
	return RETURN_CR;
}


uart_command_return_t uart_command_send_r11bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags) {
	struct can_tx_element tx_element;

	// check if CAN controller is in reset mode or busy
	if (!can_flags->bus_on || can_flags->tx_busy) {
		return RETURN_ERROR;
	}
	// check valid cmd length (only 5 bytes for RTR)
	if (cmd_len != 5) {
		return RETURN_ERROR;
	}

	can_get_tx_buffer_element_defaults(&tx_element);

	// store ID
	uint32_t id = 0;
	id = ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);

	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id) | CAN_TX_ELEMENT_T0_RTR;

	// store data length
	tx_element.T1.bit.DLC = ascii2byte(++cmd_buf_pntr);

	// if transmit buffer was empty send message
	return transmit_CAN(can_module, &tx_element);
}

uart_command_return_t uart_command_send_11bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags) {
	struct can_tx_element tx_element;

	// check if CAN controller is in reset mode or busy
	if (!can_flags->bus_on || can_flags->tx_busy) {
		return RETURN_ERROR;
	}


	if ((cmd_len < 5) || (cmd_len > 21)) {
		return RETURN_ERROR;    // check valid cmd length
	}

	can_get_tx_buffer_element_defaults(&tx_element);

	// store ID
	uint32_t id = 0;
	id = ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);

	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id);

	// store data length
	tx_element.T1.bit.DLC = ascii2byte(++cmd_buf_pntr);
	// check number of data bytes supplied against data length byte
	if (tx_element.T1.bit.DLC != ((cmd_len - 5) / 2)) {
		return RETURN_ERROR;
	}

	// check for valid length
	if (tx_element.T1.bit.DLC > 8) {
		return RETURN_ERROR;
	} else { // store data
		for (uint8_t i = 0; i < tx_element.T1.bit.DLC; i++) {
			cmd_buf_pntr++;

			tx_element.data[i] = ascii2byte(cmd_buf_pntr);
			tx_element.data[i] <<=  4;
			cmd_buf_pntr++;
			tx_element.data[i] += ascii2byte(cmd_buf_pntr);
		}
	}
	// if transmit buffer was empty send message
	return transmit_CAN(can_module, &tx_element);
}

uart_command_return_t uart_command_send_r29bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags) {
	struct can_tx_element tx_element;

	// check if CAN controller is in reset mode or busy
	if (!can_flags->bus_on || can_flags->tx_busy) {
		return RETURN_ERROR;
	}

	if (cmd_len != 10) {
		return RETURN_ERROR;    // check valid cmd length
	}

	can_get_tx_buffer_element_defaults(&tx_element);

	// store ID
	uint32_t id = 0;
	id = ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);

	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_EXTENDED_ID(id) | CAN_TX_ELEMENT_T0_XTD | CAN_TX_ELEMENT_T0_RTR;

	// store data length
	tx_element.T1.bit.DLC = ascii2byte(++cmd_buf_pntr);

	// if transmit buffer was empty send message
	return transmit_CAN(can_module, &tx_element);
}


uart_command_return_t uart_command_send_29bit_id(struct can_module *can_module, uint8_t cmd_len, uint8_t *cmd_buf_pntr, can_flags_t *can_flags) {
	struct can_tx_element tx_element;

	// check if CAN controller is in reset mode or busy
	if (!can_flags->bus_on || can_flags->tx_busy) {
		return RETURN_ERROR;
	}

	if ((cmd_len < 10) || (cmd_len > 26)) {
		return RETURN_ERROR;    // check valid cmd length
	}
	can_get_tx_buffer_element_defaults(&tx_element);

	// store ID
	uint32_t id = 0;
	id = ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);
	id <<= 4;
	id += ascii2byte(++cmd_buf_pntr);

	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_EXTENDED_ID(id) | CAN_TX_ELEMENT_T0_XTD;

	// store data length
	tx_element.T1.bit.DLC = ascii2byte(++cmd_buf_pntr);

	// check number of data bytes supplied against data lenght byte
	if (tx_element.T1.bit.DLC != ((cmd_len - 10) / 2)) {
		return RETURN_ERROR;
	}

	// check for valid length
	if (tx_element.T1.bit.DLC > 8) {
		return RETURN_ERROR;
	} else { // store data
		for (uint8_t i = 0; i < tx_element.T1.bit.DLC; i++) {
			cmd_buf_pntr++;
			tx_element.data[i] = ascii2byte(cmd_buf_pntr);
			tx_element.data[i] <<=  4;
			cmd_buf_pntr++;
			tx_element.data[i] += ascii2byte(cmd_buf_pntr);
		}
	}
	// if transmit buffer was empty send message
	return transmit_CAN(can_module, &tx_element);
}

