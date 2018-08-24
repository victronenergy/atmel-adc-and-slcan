//
// Created by stekreis on 14.03.18.
//

#include <samc21_usbcan/samc21_usbcan.h>
#include "uart_setup.h"

void configure_log_uart(usart_module_t *usart_module) {
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);
    config_usart.baudrate = 57600;
    config_usart.generator_source = GCLK_GENERATOR_1;

	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	while (usart_init(usart_module, EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {}
    usart_enable(usart_module);
}

void configure_usbcan0(usart_module_t *usart_module) {
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);
    config_usart.baudrate = 921600;//115200;
    config_usart.generator_source = GCLK_GENERATOR_1;
	config_usart.mux_setting = USBCAN0_UART_MUX_SETTING;
	config_usart.pinmux_pad0 = USBCAN0_UART_PINMUX_PAD0;
	config_usart.pinmux_pad1 = USBCAN0_UART_PINMUX_PAD1;
	config_usart.pinmux_pad2 = USBCAN0_UART_PINMUX_PAD2;
	config_usart.pinmux_pad3 = USBCAN0_UART_PINMUX_PAD3;
	while (usart_init(usart_module, USBCAN0_UART_MODULE, &config_usart) != STATUS_OK) {}
    usart_enable(usart_module);
}


void configure_usbcan1(usart_module_t *usart_module) {
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 115200;
	config_usart.generator_source = GCLK_GENERATOR_1;
	config_usart.mux_setting = USBCAN1_UART_MUX_SETTING;
	config_usart.pinmux_pad0 = USBCAN1_UART_PINMUX_PAD0;
	config_usart.pinmux_pad1 = USBCAN1_UART_PINMUX_PAD1;
	config_usart.pinmux_pad2 = USBCAN1_UART_PINMUX_PAD2;
	config_usart.pinmux_pad3 = USBCAN1_UART_PINMUX_PAD3;
	while (usart_init(usart_module, USBCAN1_UART_MODULE, &config_usart) != STATUS_OK) {}
	usart_enable(usart_module);
}


void configure_can0(struct can_module *can_instance){

	/* Set up the CAN TX/RX pins */
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);
	pin_config.mux_position = CAN0_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN0_TX_PIN, &pin_config);
	pin_config.mux_position = CAN0_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN0_RX_PIN, &pin_config);
	/* Initialize the module. */
	struct can_config config_can;
	can_get_config_defaults(&config_can);
	config_can.nonmatching_frames_action_standard = CAN_NONMATCHING_FRAMES_FIFO_0;
	config_can.nonmatching_frames_action_extended = CAN_NONMATCHING_FRAMES_FIFO_1;
	config_can.remote_frames_standard_reject = true;
	config_can.remote_frames_extended_reject = true;
	config_can.extended_id_mask = 0x00000000;

	can_init(can_instance, CAN0_MODULE, &config_can);

	can_start(can_instance);
	/* Enable interrupts for this CAN module */
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
	can_enable_interrupt(can_instance, CAN_PROTOCOL_ERROR_ARBITRATION | CAN_PROTOCOL_ERROR_DATA);


}


void configure_can1(struct can_module *can_instance){

	/* Set up the CAN TX/RX pins */
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);
	pin_config.mux_position = CAN1_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN1_TX_PIN, &pin_config);
	pin_config.mux_position = CAN1_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN1_RX_PIN, &pin_config);
	/* Initialize the module. */
	struct can_config config_can;
	can_get_config_defaults(&config_can);
	config_can.nonmatching_frames_action_standard = CAN_NONMATCHING_FRAMES_FIFO_0;
	config_can.nonmatching_frames_action_extended = CAN_NONMATCHING_FRAMES_FIFO_1;
	config_can.remote_frames_standard_reject = true;
	config_can.remote_frames_extended_reject = true;
	config_can.extended_id_mask = 0x00000000;

	can_init(can_instance, CAN1_MODULE, &config_can);

	can_start(can_instance);
	/* Enable interrupts for this CAN module */
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN1);
	can_enable_interrupt(can_instance, CAN_PROTOCOL_ERROR_ARBITRATION | CAN_PROTOCOL_ERROR_DATA);


}
