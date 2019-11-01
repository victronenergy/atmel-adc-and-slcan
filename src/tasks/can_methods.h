//
// Created by Felix Hübner on 2019-11-01.
//

#ifndef SAMC_FREERTOS_CAN_METHODS_H
#define SAMC_FREERTOS_CAN_METHODS_H

bool check_and_transfer_can_message_to_uart(struct can_module *const can_module, struct can_rx_element_fifo_0 *rx_message, uint8_t cantask_id, uint8_t *sequence_counter);

#endif //SAMC_FREERTOS_CAN_METHODS_H
