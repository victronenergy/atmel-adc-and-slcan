//
// Created by Felix Hübner on 29.04.18.
//

#ifndef SAMC_FREERTOS_CUSTOM_BOARD_H
#define SAMC_FREERTOS_CUSTOM_BOARD_H

/*! \name Base Boards
 */
//! @{
#define CUSTOM_BOARD_TEMPLATE				1  //!< custom board template to adapt to any new board.
#define CUSTOM_BOARD_1						2  //!< the first custom board, change name to your needs.

#if CUSTOM_BOARD == CUSTOM_BOARD_TEMPLATE
# include  "custom_board_template.h"
#elif CUSTOM_BOARD == CUSTOM_BOARD_1 	//change name to your needs!
# include  "samc21_usbcan.h"			//change name to your needs!
#else
#  error "No known Unipoin board defined"
#endif
#endif //SAMC_FREERTOS_CUSTOM_BOARD_H
