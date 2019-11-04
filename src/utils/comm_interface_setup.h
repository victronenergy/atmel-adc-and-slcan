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

/******** Prototypes ********/
/**
 * configure debug interface
 * @param usart_module usart module
 */
void configure_log_uart(usart_module_t *usart_module);
/**
 * configure usb uart interface associated with can0
 * @param usart_module usart module
 */
void configure_uart_can0(usart_module_t *usart_module);

#endif //SAMC_FREERTOS_COMM_INIT_H
