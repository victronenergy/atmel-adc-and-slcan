//
// Created by stekreis on 14.03.18.
//

#include <samc21_slcan_adc.h>
#include "comm_interface_setup.h"

// debug interface
void configure_log_uart(usart_module_t *usart_module) {
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);
    config_usart.baudrate = 115200;
    config_usart.generator_source = GCLK_GENERATOR_1;
	config_usart.mux_setting = DEBUG_UART_MUX_SETTING;
	config_usart.pinmux_pad0 = DEBUG_UART_PINMUX_PAD0;
	config_usart.pinmux_pad1 = DEBUG_UART_PINMUX_PAD1;
	config_usart.pinmux_pad2 = DEBUG_UART_PINMUX_PAD2;
	config_usart.pinmux_pad3 = DEBUG_UART_PINMUX_PAD3;
	while (usart_init(usart_module, DEBUG_UART_MODULE, &config_usart) != STATUS_OK) {}
    usart_enable(usart_module);
}

// usb uart interface associated with can0
void configure_uart_can0(usart_module_t *usart_module) {
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);
    config_usart.baudrate = 1500000;
    config_usart.generator_source = GCLK_GENERATOR_1;
	config_usart.mux_setting = USBCAN0_UART_MUX_SETTING;
	config_usart.pinmux_pad0 = USBCAN0_UART_PINMUX_PAD0;
	config_usart.pinmux_pad1 = USBCAN0_UART_PINMUX_PAD1;
	config_usart.pinmux_pad2 = USBCAN0_UART_PINMUX_PAD2;
	config_usart.pinmux_pad3 = USBCAN0_UART_PINMUX_PAD3;

	while (usart_init(usart_module, USBCAN0_UART_MODULE, &config_usart) != STATUS_OK) {}
    usart_enable(usart_module);
}
