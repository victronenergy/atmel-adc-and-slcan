#ifndef CERBO_GX_SLCAN_ADC_H_INCLUDED
#define CERBO_GX_SLCAN_ADC_H_INCLUDED

#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

void system_board_init(void);

/** Name string macro */
#define BOARD_NAME                "Cerbo GX"

/** \name Resonator definitions
 *  @{ */
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS  0 /* Not Mounted */
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625
/** @} */

/**
 * \name HW Rev detection Pins
 */
#define HW_REV_DETECTION_0		PIN_PA18
#define HW_REV_DETECTION_1		PIN_PA19
#define HW_REV_DETECTION_2		PIN_PA20

/** \name LED definitions
 *
 */
#define HW_REV_1_LEDPIN_C21_GREEN		PIN_PA12
#define HW_REV_1_LEDPIN_C21_RED			PIN_PA13
#define HW_REV_2_LEDPIN_C21_GREEN		PIN_PA10
#define HW_REV_2_LEDPIN_C21_RED			PIN_PA11
#define LED_ACTIVE              false
#define LED_INACTIVE            !LED_ACTIVE
/** @} */


/** \name Extension header #1 UART definitions
 *  @{
 */
#define USBCAN0_UART_MODULE       SERCOM3
#define USBCAN0_UART_MUX_SETTING  USART_RX_1_TX_0_RTS_2_CTS_3
#define USBCAN0_UART_PINMUX_PAD0  PINMUX_PA22C_SERCOM3_PAD0
#define USBCAN0_UART_PINMUX_PAD1  PINMUX_PA23C_SERCOM3_PAD1
#define USBCAN0_UART_PINMUX_PAD2  PINMUX_PA24C_SERCOM3_PAD2
#define USBCAN0_UART_PINMUX_PAD3  PINMUX_PA25C_SERCOM3_PAD3
#define USBCAN0_UART_DMAC_ID_TX   SERCOM3_DMAC_ID_TX
#define USBCAN0_UART_DMAC_ID_RX   SERCOM3_DMAC_ID_RX
/** @} */

/** \name Embedded debugger CDC Gateway USART interface definitions
 * @{
 */
#define DEBUG_UART_MODULE				SERCOM1
#define DEBUG_UART_MUX_SETTING			USART_RX_1_TX_0_XCK_1
#define DEBUG_UART_PINMUX_PAD0			PINMUX_PA16C_SERCOM1_PAD0
#define DEBUG_UART_PINMUX_PAD1			PINMUX_PA17C_SERCOM1_PAD1
#define DEBUG_UART_PINMUX_PAD2			PINMUX_UNUSED
#define DEBUG_UART_PINMUX_PAD3			PINMUX_UNUSED
#define DEBUG_UART_DMAC_ID_TX			SERCOM1_DMAC_ID_TX
#define DEBUG_UART_DMAC_ID_RX			SERCOM1_DMAC_ID_RX
/** @} */

/** \name CAN0 interface definitions
 * @{
 */
#define CAN0_MODULE              CAN0
#define CAN0_TX_PIN				 PIN_PB22G_CAN0_TX
#define CAN0_TX_MUX_SETTING      MUX_PB22G_CAN0_TX
#define CAN0_RX_PIN              PIN_PB23G_CAN0_RX
#define CAN0_RX_MUX_SETTING      MUX_PB23G_CAN0_RX
/** @} */


/* slave sercom pinmux setting */
#define HW_REV_1_ADC_I2C_SLAVE_MODULE			SERCOM0
#define HW_REV_1_ADC_I2C_SLAVE_SDA_PINMUX		PINMUX_PA08C_SERCOM0_PAD0
#define HW_REV_1_ADC_I2C_SLAVE_SCK_PINMUX		PINMUX_PA09C_SERCOM0_PAD1
#define HW_REV_1_ADC_I2C_SLAVE_DMA_RX_TRIGGER	SERCOM0_DMAC_ID_RX
#define HW_REV_1_ADC_I2C_SLAVE_DMA_TX_TRIGGER	SERCOM0_DMAC_ID_TX
#define HW_REV_2_ADC_I2C_SLAVE_MODULE			SERCOM2
#define HW_REV_2_ADC_I2C_SLAVE_SDA_PINMUX		PINMUX_PA12C_SERCOM2_PAD0
#define HW_REV_2_ADC_I2C_SLAVE_SCK_PINMUX		PINMUX_PA13C_SERCOM2_PAD1
#define HW_REV_2_ADC_I2C_SLAVE_DMA_RX_TRIGGER	SERCOM0_DMAC_ID_RX
#define HW_REV_2_ADC_I2C_SLAVE_DMA_TX_TRIGGER	SERCOM0_DMAC_ID_TX


/**
 * \brief Turns off the specified LEDs.
 *
 * \param led_gpio LED to turn off (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Off(led_gpio)     port_pin_set_output_level(led_gpio,true)

/**
 * \brief Turns on the specified LEDs.
 *
 * \param led_gpio LED to turn on (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_On(led_gpio)      port_pin_set_output_level(led_gpio,false)

/**
 * \brief Toggles the specified LEDs.
 *
 * \param led_gpio LED to toggle (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Toggle(led_gpio)  port_pin_toggle_output_level(led_gpio)

#ifdef __cplusplus
}
#endif

#endif  /* CERBO_GX_SLCAN_ADC_H_INCLUDED */
