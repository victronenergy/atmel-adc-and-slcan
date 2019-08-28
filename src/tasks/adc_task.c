//
// Created by Felix Hübner on 2019-05-20.
//

#include "adc_task.h"
#include "log.h"
#include "i2c_vitual_eeprom.h"
#include "adc_methods.h"

/*
 * Defines
 */

#define SLAVE_ADDRESS					0x12

#ifndef SW_VERSION
	#define SW_VERSION -1
#endif

/*
 * Prototypes
 */
void vAdcTask(void *pvParameters);
void readSerialNumber(uint8_t serial_no[]);


void readSerialNumber(uint8_t serial_no[]) {
	// copy serialnumber to ram!
	// Word 0
	uint32_t tmp =  *((uint32_t *) 0x0080A00C);
	serial_no[0] = (uint8_t) (tmp >> 24);
	serial_no[1] = (uint8_t) (tmp >> 16);
	serial_no[2] = (uint8_t) (tmp >> 8);
	serial_no[3] = (uint8_t) (tmp >> 0);

	// Word 1-3
	for (uint8_t i = 0; i<9;i+=4) {
		tmp = *((uint32_t *) (0x0080A040+i));
		serial_no[4+(i)] = (uint8_t) (tmp >> 24);
		serial_no[5+(i)] = (uint8_t) (tmp >> 16);
		serial_no[6+(i)] = (uint8_t) (tmp >> 8);
		serial_no[7+(i)] = (uint8_t) (tmp >> 0);
	}
}



void vAdcTask(void *pvParameters){
	port_pin_set_output_level(PIN_PA15, true);

	configASSERT(pvParameters);
	configASSERT(((adctask_params *)pvParameters)->adc_instance0);
	configASSERT(((adctask_params *)pvParameters)->adc_instance1);
	configASSERT(((adctask_params *)pvParameters)->dma_resource0);
	configASSERT(((adctask_params *)pvParameters)->dma_resource1);
	configASSERT(((adctask_params *)pvParameters)->i2c_instance);

	adc_module_t  *adc_instance0 = NULL;
	adc_module_t  *adc_instance1 = NULL;
	dma_resource_t *dma_adc_resource0 = NULL;
	dma_resource_t *dma_adc_resource1 = NULL;

	adc_instance0 = ((adctask_params *)pvParameters)->adc_instance0;
	adc_instance1 = ((adctask_params *)pvParameters)->adc_instance1;
	dma_adc_resource0 = ((adctask_params *)pvParameters)->dma_resource0;
	dma_adc_resource1 = ((adctask_params *)pvParameters)->dma_resource1;

	configASSERT(adc_instance0);
	configASSERT(adc_instance1);
	configASSERT(dma_adc_resource0);
	configASSERT(dma_adc_resource1);

	ulog_s("start ADC Task ...\r\n");

	adc_i2c_eeprom_u eeprom_data = {.s.adc_counter = 0xFFFF, .s.adc_tank_channel ={{0}}, .s.adc_temp_channel = {{0}}, .s.preamble = {'V', 'E'}, .s.sw_rev=SW_VERSION, .s.serial_no = {0}};
	eeprom_data.s.config_size = (EEPROM_USED_SIZE - ((uint32_t) eeprom_data.s.serial_no - (uint32_t) &eeprom_data));
	set_eeprom_data_pointer(&eeprom_data);

	// read unique serial number from device info ram-eeprom
	readSerialNumber(eeprom_data.s.serial_no);

	// local i2c data instance
	struct i2c_slave_packet i2c_packet;
	set_i2c_slave_data_packet(&i2c_packet);

	DmacDescriptor dma_desc[4]= {SECTION_DMAC_DESCRIPTOR};
	port_pin_set_output_level(PIN_PA15, false);

	taskENTER_CRITICAL(  );
	// configure adc with dma
	{
		uint32_t dest_ptr_addr[4];
		dest_ptr_addr[0] = (uint32_t) &(eeprom_data.s.adc_tank_channel[0]);
		dest_ptr_addr[1] = (uint32_t) &(eeprom_data.s.adc_tank_channel[1]);
		dest_ptr_addr[2] = (uint32_t) &(eeprom_data.s.adc_temp_channel[0]);
		dest_ptr_addr[3] = (uint32_t) &(eeprom_data.s.adc_temp_channel[1]);

		adc_module_t *adc_modules[2];
		adc_modules[0] = adc_instance0;
		adc_modules[1] = adc_instance1;

		dma_resource_t *dma_resources[2];
		dma_resources[0] = dma_adc_resource0;
		dma_resources[1] = dma_adc_resource1;

		configure_adc_dma(dma_resources, adc_modules, dma_desc, dest_ptr_addr);
	}

	// configure i2c slave
	configure_i2c_slave(((adctask_params *)pvParameters)->i2c_instance,SLAVE_ADDRESS);
	taskEXIT_CRITICAL(  );


	while(1) {
		port_pin_set_output_level(PIN_PA15, true);
		// idle

		if (!is_adc_busy()) {

			port_pin_set_output_level(PIN_PA15, false);
			port_pin_set_output_level(PIN_PA15, true);
/*			uint8_t k = (uint8_t) (eeprom_data.s.adc_counter & 0x01);

				ulog_s("\r\nbuffer: ");
				xlog(&k,1);

				ulog_s("\ttank sensor readings: ");
				for(uint8_t i = 0; i < 4; i++) {
					xlog_short(&eeprom_data.s.adc_tank_channel[k][i]);
					ulog_s(" ");
				}

				ulog_s("\ttemp sensor readings: ");
				for(uint8_t i = 0; i < 4; i++) {
					xlog_short(&eeprom_data.s.adc_temp_channel[k][i]);
					ulog_s(" ");
				}

				ulog_s("\tcnt: ");
				xlog_short(&eeprom_data.s.adc_counter);*/

			port_pin_set_output_level(PIN_PA15, false);
			port_pin_set_output_level(PIN_PA15, true);

			port_pin_set_output_level(PIN_PA14, false);
			port_pin_set_output_level(PIN_PA19, false);
			port_pin_set_output_level(PIN_PA14, true);
			port_pin_set_output_level(PIN_PA19, true);
			adc_trigger_new_conv();
		}

		port_pin_set_output_level(PIN_PA15, false);
		vTaskDelay(500);
	}
}

TaskHandle_t vCreateAdcTask(adctask_params *params) {

	BaseType_t xReturned;
	TaskHandle_t xHandle = NULL;

	ulog_s("creating ADC Task\r\n");

	/* Create the task, storing the handle. */
	xReturned = xTaskCreate(
			vAdcTask,       /* Function that implements the task. */
			"aTask",          /* Text name for the task. */
			200,      /* Stack size in words, not bytes. */
			params,    /* Parameter passed into the task. */
			tskIDLE_PRIORITY,/* Priority at which the task is created. */
			&xHandle);      /* Used to pass out the created task's handle. */

	if (xReturned == pdPASS) {
		ulog_s("successfully created ADC Task\r\n");

		/* The task was created.  Use the task's handle to delete the task. */
		//vTaskDelete( xHandle );
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;


}