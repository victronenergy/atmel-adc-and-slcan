//
// Created by stekreis on 14.03.18.
//

#include "uart_setup.h"


void configure_uplink_RS485(usart_module_t *usart_module, uint32_t baudrate);
void configure_vebus_uart(usart_module_t *usart_module);

/*
void configure_log_uart(usart_module_t *usart_module) {
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);
    config_usart.baudrate = 57600;
    config_usart.generator_source = GCLK_GENERATOR_1;
    config_usart.mux_setting = DEBUG_SERCOM_MUX_SETTING;
    config_usart.pinmux_pad0 = DEBUG_SERCOM_PINMUX_PAD0;
    config_usart.pinmux_pad1 = DEBUG_SERCOM_PINMUX_PAD1;
    config_usart.pinmux_pad2 = DEBUG_SERCOM_PINMUX_PAD2;
    config_usart.pinmux_pad3 = DEBUG_SERCOM_PINMUX_PAD3;
    while (usart_init(usart_module, DEBUG_MODULE, &config_usart) != STATUS_OK) {}
    usart_enable(usart_module);
}*/


void configure_log_uart(usart_module_t *usart_module) {
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);
    config_usart.baudrate = 57600;
    config_usart.generator_source = GCLK_GENERATOR_1;
    config_usart.mux_setting = EXT1_UART_SERCOM_MUX_SETTING;
    config_usart.pinmux_pad0 = EXT1_UART_SERCOM_PINMUX_PAD0;
    config_usart.pinmux_pad1 = EXT1_UART_SERCOM_PINMUX_PAD1;
    config_usart.pinmux_pad2 = EXT1_UART_SERCOM_PINMUX_PAD2;
    config_usart.pinmux_pad3 = EXT1_UART_SERCOM_PINMUX_PAD3;
    while (usart_init(usart_module, EXT1_UART_MODULE, &config_usart) != STATUS_OK) {}
    usart_enable(usart_module);
}
