//
// Created by Felix Hübner on 2019-11-01.
//

#ifndef SAMC_FREERTOS_CAN_METHODS_H
#define SAMC_FREERTOS_CAN_METHODS_H

typedef struct {
	uint16_t init_complete:1;
	uint16_t bus_on:1;
	uint16_t tx_busy:1;
} can_flags_t;

bool check_and_transfer_can_message_to_uart(struct can_module *const can_module, struct can_rx_element_fifo_0 *rx_message, uint8_t cantask_id, uint8_t *sequence_counter);

void reset_can_errorflags(can_flags_t *CAN_flags);
void setup_can_instance(struct can_module *can_module, Can *can_hw, uint32_t bitrate);
uint8_t transmit_CAN(struct can_module *const can_module, struct can_tx_element *tx_element);

#endif //SAMC_FREERTOS_CAN_METHODS_H
