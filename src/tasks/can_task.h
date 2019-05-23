//
// Created by stekreis on 25.05.18.
//

#ifndef SAMC_FREERTOS_CAN_TASK_H
#define SAMC_FREERTOS_CAN_TASK_H
#include "conf_can.h"
#include "can.h"
#include "log.h"

typedef struct {
	uint8_t task_id;
	usart_module_t *usart_instance;
	Can *can_instance;
} cantask_params;


TaskHandle_t vCreateCanTask(cantask_params *params);

#define CANTASK_ID_0 0
#define CANTASK_ID_1 1

/*
 *
 * ASF CAN example
 */

//Define CAN standard filter setting.
#define CAN_RX_STANDARD_FILTER_INDEX_0    0
#define CAN_RX_STANDARD_FILTER_INDEX_1    1
#define CAN_RX_STANDARD_FILTER_ID_0     0x00//0x45A
#define CAN_RX_STANDARD_FILTER_ID_0_BUFFER_INDEX     2
#define CAN_RX_STANDARD_FILTER_ID_1     0x00//0x469
#define CAN_RX_EXTENDED_FILTER_INDEX_0    0
#define CAN_RX_EXTENDED_FILTER_INDEX_1    1
#define CAN_RX_EXTENDED_FILTER_ID_0     0x00000000//0x100000A5
#define CAN_RX_EXTENDED_FILTER_ID_0_BUFFER_INDEX     1
#define CAN_RX_EXTENDED_FILTER_ID_1     0x00000000//0x10000096

// Define CAN standard transfer message setting.
#define CAN_TX_BUFFER_INDEX    0

#if !defined(CR)
#define CR				13	// command end tag (ASCII CR)
#endif
/*
#if !defined(ERROR)
#define ERROR			7	// error tag (ASCII BEL)
#endif

#if !defined(NO_RETURN)
#define NO_RETURN		0	// will be used to indicate that no answer should be send!
#endif*/



#define TIME_STAMP_TICK 1000	// microseconds

/*
	define command receive buffer length
	minimum length is define as follow:
	1 byte	: command identifier
	8 byte	: CAN identifier (for both 11bit and 29bit ID)
	1 byte	: CAN data length (0-8 byte)
	2*8 byte: CAN data (one data byte is send as two ASCII chars)
	1 byte  : [CR] command end tag
	---
	27 byte
*/
#define CMD_BUFFER_LENGTH  32

// calculate timer0 overflow value
#define OCR_VALUE ((unsigned char)((unsigned long)(TIME_STAMP_TICK) / (1000000L / (float)((unsigned long)F_CPU / 64L))))

#endif //SAMC_FREERTOS_CAN_TASK_H
