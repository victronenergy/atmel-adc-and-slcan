/**
 * \file
 *
 * \brief SAM C21 Xplained Pro board definition
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef SAMC_FREERTOS_CUSTOM_BOARD_TEMPLATE_H
#define SAMC_FREERTOS_CUSTOM_BOARD_TEMPLATE_H

#include <compiler.h>
#include <usart.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup group_common_boards
 * \defgroup samc21_xplained_pro_group SAM C21 Xplained Pro board
 *
 * @{
 */

void system_board_init(void);

/**
 * \defgroup samc21_xplained_pro_features_group Features
 *
 * Symbols that describe features and capabilities of the board.
 *
 * @{
 */

/** Name string macro */
#define BOARD_NAME					"GRIDLESS_CONV_VE_BUS_A3"
#define SW_VERSION					"0.8.329" //stands for 8=2018 3=(march) 19=(19th day)

/** \name Resonator definitions
 *  @{ */
#define BOARD_FREQ_SLCK_XTAL      0 /* Not Mounted */
#define BOARD_FREQ_SLCK_BYPASS    0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_XTAL    18432000 
#define BOARD_FREQ_MAINCK_BYPASS  18432000
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625
/** @} */

/** \name SW0 definitions
 *  @{ */
#define SW0_PIN                   PIN_PA27
#define SW0_ACTIVE                false
#define SW0_INACTIVE              !SW0_ACTIVE
#define SW0_EIC_PIN               PIN_PA27A_EIC_EXTINT15
#define SW0_EIC_MUX               MUX_PA27A_EIC_EXTINT15
#define SW0_EIC_PINMUX            PINMUX_PA27A_EIC_EXTINT15
#define SW0_EIC_LINE              15
/** @} */

/**
 * \name HW Rev detection Pins
 */
#define HW_REV_DETECTION_0			PIN_PA02
#define HW_REV_DETECTION_1			PIN_PA03
#define HW_REV_DETECTION_2			PIN_PA04

/**
 * \name SERIALNUMBER
 */

#define HW_SERIAL_WORD_0			0x0080A00C
#define HW_SERIAL_WORD_1			0x0080A040
#define HW_SERIAL_WORD_2			0x0080A044
#define HW_SERIAL_WORD_3			0x0080A048

/**
 * \name Button #0 definitions
 *
 * Wrapper macros for SW0, to ensure common naming across all Xplained Pro
 * boards.
 *
 *  @{ */
#define BUTTON_0_NAME             "SW0"
#define BUTTON_0_PIN              SW0_PIN
#define BUTTON_0_ACTIVE           SW0_ACTIVE
#define BUTTON_0_INACTIVE         SW0_INACTIVE
#define BUTTON_0_EIC_PIN          SW0_EIC_PIN
#define BUTTON_0_EIC_MUX          SW0_EIC_MUX
#define BUTTON_0_EIC_PINMUX       SW0_EIC_PINMUX
#define BUTTON_0_EIC_LINE         SW0_EIC_LINE
/** @} */

/** Number of on-board buttons */
#define BUTTON_COUNT 1

/** \name UPLINK USART RS485 definitions
 *  @{
 */
#define UPLINK_RS485_MODULE              SERCOM2
#define UPLINK_RS485_SERCOM_MUX_SETTING  USART_RX_3_TX_0_XCK_1_TE_2
#define UPLINK_RS485_SERCOM_PINMUX_PAD0  PINMUX_PA08D_SERCOM2_PAD0
#define UPLINK_RS485_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define UPLINK_RS485_SERCOM_PINMUX_PAD2  PINMUX_PA10D_SERCOM2_PAD2
#define UPLINK_RS485_SERCOM_PINMUX_PAD3  PINMUX_PA11D_SERCOM2_PAD3
#define UPLINK_RS485_SERCOM_DMAC_ID_TX   SERCOM2_DMAC_ID_TX
#define UPLINK_RS485_SERCOM_DMAC_ID_RX   SERCOM2_DMAC_ID_RX
/** @} */


/** \name VEBUS UART definitions
 *  @{
 */
#define VEBUS_UART_MODULE              SERCOM0
#define VEBUS_UART_SERCOM_MUX_SETTING  USART_RX_1_TX_2_XCK_3
#define VEBUS_UART_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define VEBUS_UART_SERCOM_PINMUX_PAD1  PINMUX_PA05D_SERCOM0_PAD1
#define VEBUS_UART_SERCOM_PINMUX_PAD2  PINMUX_PA06D_SERCOM0_PAD2
#define VEBUS_UART_SERCOM_PINMUX_PAD3  PINMUX_UNUSED
#define VEBUS_UART_SERCOM_DMAC_ID_TX   SERCOM0_DMAC_ID_TX
#define VEBUS_UART_SERCOM_DMAC_ID_RX   SERCOM0_DMAC_ID_RX
/** @} */

/** \name UPLINK USART RS485 definitions
 *  @{
 */
#define DEBUG_MODULE              SERCOM1
#define DEBUG_SERCOM_MUX_SETTING  USART_RX_1_TX_0_XCK_1_TE_2
#define DEBUG_SERCOM_PINMUX_PAD0  PINMUX_PA16C_SERCOM1_PAD0
#define DEBUG_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define DEBUG_SERCOM_PINMUX_PAD2  PINMUX_UNUSED
#define DEBUG_SERCOM_PINMUX_PAD3  PINMUX_UNUSED
#define UPLINK_RS485_SERCOM_DMAC_ID_TX   SERCOM2_DMAC_ID_TX
#define UPLINK_RS485_SERCOM_DMAC_ID_RX   SERCOM2_DMAC_ID_RX
/** @} */

#ifdef __cplusplus
}
#endif

#endif  /* SAMC_FREERTOS_CUSTOM_BOARD_TEMPLATE_H */
