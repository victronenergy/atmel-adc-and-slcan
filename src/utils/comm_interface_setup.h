//
// Created by stekreis on 14.03.18.
//

#ifndef SAMC_FREERTOS_COMM_INIT_H
#define SAMC_FREERTOS_COMM_INIT_H

#include <asf.h>
#include <stdint.h>
#include <sys/types.h>

#include <log.h>
#include <can.h>

void configure_log_uart(usart_module_t *usart_module);
void configure_usbcan0(usart_module_t *usart_module);
void configure_usbcan1(usart_module_t *usart_module);
void configure_can0(struct can_module *can_instance);
void configure_can1(struct can_module *can_instance);

#endif //SAMC_FREERTOS_COMM_INIT_H