//
// Created by Felix Hübner on 2019-11-01.
//

#ifndef SAMC_FREERTOS_CAN_METHODS_H
#define SAMC_FREERTOS_CAN_METHODS_H

/******** Typedef ********/
typedef struct {
	uint16_t init_complete:1;
	uint16_t bus_on:1;
	uint16_t tx_busy:1;
} can_flags_t;

/******** Prototypes ********/
/**
 * Read message from CAN and send as slcan message via uart
 * @param can_module can module we want to read from
 * @param cantask_id id of the cantask
 * @param sequence_counter pointer to the sequence_counter (DEBUG)
 * @return return true if a new message was transferred from CAN to UART
 */
bool check_and_transfer_can_message_to_uart(struct can_module *const can_module, uint8_t cantask_id, uint8_t *sequence_counter);

/**
 * method usually resets the errorflags in the SJA1000, here we to not more than reset the ERROR LED
 * @param can_flags pointer to the error flags, currently not used
 */
void reset_can_errorflags(can_flags_t *can_flags);

/**
 * setup and open the CAN interface
 * @param can_module can module we want to setup and start
 * @param can_hw pointer to the hardware of the can interface
 * @param bitrate bitrate the can hardware should run at
 */
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate);

/**
 * start sending a can message (transfer a tx_element to the tx_fifo and initiate the sendout)
 * @param can_module can module where we want send the tx_element
 * @param tx_element message to send
 * @return return ERROR if case the copy to the hardware was not successful, ERROR_BUSY if the hardware is still busy sending, or otherwise NO_RETURN
 */
uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element);

/**
 * test the interrupt flags to produce a SJA1000 compatible status-flag 8bit mask (in can_flags->status)
 * restarts the canbus if a bus-off interrupt is found
 * @param can_module can module we want to setup and start
 * @param can_instance pointer to the hardware of the can interface
 * @param can_flags pointer to the can flags and status structure
 * @param can_bitrate pointer to the currently configured can bitrate
 */
void can_error_handling(struct can_module *can_module, Can *can_instance, can_flags_t *can_flags, uint32_t *can_bitrate);

#endif //SAMC_FREERTOS_CAN_METHODS_H
