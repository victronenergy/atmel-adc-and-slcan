//
// Created by Felix Hübner on 2019-05-20.
//

#include "adc_task.h"
#include "log.h"
#include "i2c_vitual_eeprom.h"
#include "adc_methods.h"
#include "board_setup.h"
#include "stack_task.h"

/******** Defines ********/
#define SLAVE_ADDRESS		0x12

#ifndef SW_VERSION
	#define SW_VERSION 		-1
#endif

/******** Internal Prototypes ********/
void vAdcTask(void *pvParameters);


/******** Methods ********/
/**
 * Setup and handles the adc conversion and initialize the i2c_eeprom_emulation
 *
 * @param pvParameters contains the pointers to the modules and structs that should not live inside the task, but as
 * local variable in main.
 */
void vAdcTask(void *pvParameters){
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

	TaskHandle_t xhandle = xTaskGetCurrentTaskHandle();
	adc_i2c_eeprom_u eeprom_data = {.s.adc_counter = 0xFFFF,
								 	.s.adc_channels ={{0}},
								 	.s.preamble = {'V', 'E'},
								 	.s.sw_rev=(uint16_t) SW_VERSION,
								 	.s.hw_rev= hw_rev,
								 	.s.serial_no = {0}};
	eeprom_data.s.config_size = (EEPROM_USED_SIZE - ((uint32_t) eeprom_data.s.serial_no - (uint32_t) &eeprom_data));
	set_eeprom_data_pointer(&eeprom_data);

	// read unique serial number from device info ram-eeprom
	readSerialNumber(eeprom_data.s.serial_no);

	// local i2c data instance
	struct i2c_slave_packet i2c_packet;
	set_i2c_slave_data_packet(&i2c_packet);

	//do startup calibration
	uint16_t gain_corrections[2] = {0};
	int16_t offset_corrections[2] = {0};
	bool do_calibration = false;
	if (hw_rev >= HW_REV_2){
		adc_module_t *adc_modules[2];
		adc_modules[0] = adc_instance0;
		adc_modules[1] = adc_instance1;

		adc_perform_calibration(adc_modules, gain_corrections, offset_corrections);
		do_calibration = true;
	}

	DmacDescriptor dma_desc[4]= {SECTION_DMAC_DESCRIPTOR};

	// configure adc with dma
	{
		uint32_t dest_ptr_addr[4];
		dest_ptr_addr[0] = (uint32_t) &(eeprom_data.s.adc_channels[0][0]);
		dest_ptr_addr[1] = (uint32_t) &(eeprom_data.s.adc_channels[1][0]);
		dest_ptr_addr[2] = (uint32_t) &(eeprom_data.s.adc_channels[0][5]);
		dest_ptr_addr[3] = (uint32_t) &(eeprom_data.s.adc_channels[1][5]);

		adc_module_t *adc_modules[2];
		adc_modules[0] = adc_instance0;
		adc_modules[1] = adc_instance1;

		dma_resource_t *dma_resources[2];
		dma_resources[0] = dma_adc_resource0;
		dma_resources[1] = dma_adc_resource1;

		configure_adc_dma(dma_resources, adc_modules, dma_desc, dest_ptr_addr, do_calibration, gain_corrections, offset_corrections);
	}

	// configure i2c slave
	configure_i2c_slave(((adctask_params *)pvParameters)->i2c_instance,SLAVE_ADDRESS);

	ulog_s("start ADC Task ...\r\n");

	while(1) {
		// idle
		if (!is_adc_busy()) {
			adc_trigger_new_conv();
		}
		vTaskDelay(500);

		//call wdt reset method
		task_wdt_reset(xhandle);
	}
}


/**
 * will be called from main to set up the adc task, will suspend the task after creation
 *
 *
 * @param params contains the pointers to the modules and structs that should not life inside the task, but as
 * local variable in main.
 * @return NULL or on success a valid handle
 */
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
		vTaskSuspend(xHandle);
		return xHandle;
	}
	return NULL;


}