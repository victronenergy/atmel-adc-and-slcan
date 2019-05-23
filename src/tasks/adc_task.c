//
// Created by Felix Hübner on 2019-05-20.
//


#include <adc.h>
#include <adc_sam_l_c/adc_feature.h>
#include <dma.h>
#include <adc_callback.h>
#include "adc_task.h"
#include "log.h"

/*
 * Prototypes
 */
void vAdcTask(void *pvParameters);


void dma_callback_transfer_done0(struct dma_resource *const resource);
void dma_callback_transfer_done1(struct dma_resource *const resource);
void configure_adc(adc_module_t *adc_instance0, adc_module_t *adc_instance1);
void configure_dma_resource(struct dma_resource *resource, uint8_t dma_peripheral_trigger);
void setup_transfer_descriptor(DmacDescriptor *descriptor, DmacDescriptor *followingDescriptior, uint32_t source_addr, uint32_t dest_addr);

typedef struct {
	const uint8_t preamble[2];
	const uint16_t config_size;
	uint8_t serial_no[10];
	const uint16_t sw_rev;
	uint16_t gain_correction;
	uint16_t offset_correction;
	uint16_t adc_tank_channel[2][4]; //double buffered, Channel 4-1
	uint16_t adc_temp_channel[2][4]; //double buffered, Channel 4-1
	uint16_t adc_counter; //counter, will be increased after each complete conversion round with buffer swap
} adc_i2c_eeprom_t;


adc_i2c_eeprom_t *eeprom_data_ptr = NULL;

void configure_adc(adc_module_t *adc_instance0, adc_module_t *adc_instance1)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);


	//configure ADC0
	config_adc.resolution		= ADC_RESOLUTION_CUSTOM;
	config_adc.clock_source		= GCLK_GENERATOR_2;
	config_adc.clock_prescaler	= ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference		= ADC_REFERENCE_INTVCC2;
	config_adc.negative_input	= ADC_NEGATIVE_INPUT_GND;
	config_adc.freerunning		= false;
	config_adc.left_adjust		= false;
	config_adc.positive_input_sequence_mask_enable = 1 << ADC_POSITIVE_INPUT_PIN4 | 1 << ADC_POSITIVE_INPUT_PIN5 | 1 << ADC_POSITIVE_INPUT_PIN6 | 1 << ADC_POSITIVE_INPUT_PIN7; //PA04-PA07
	config_adc.window.window_mode = ADC_WINDOW_MODE_DISABLE;
	config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_64;
	config_adc.divide_result = ADC_DIVIDE_RESULT_16;
	config_adc.sample_length = 10;

	adc_init(adc_instance0, ADC0, &config_adc);
	adc_enable_positive_input_sequence(adc_instance0, config_adc.positive_input_sequence_mask_enable);

	//configure ADC1
	config_adc.positive_input_sequence_mask_enable = 1 << ADC_POSITIVE_INPUT_PIN2 | 1 << ADC_POSITIVE_INPUT_PIN3 | 1 << ADC_POSITIVE_INPUT_PIN4 | 1 << ADC_POSITIVE_INPUT_PIN5; //PB02-PB03,PB08-PB09
	adc_init(adc_instance1, ADC1, &config_adc);
	adc_enable_positive_input_sequence(adc_instance1, config_adc.positive_input_sequence_mask_enable);
	adc_set_master_slave_mode(adc_instance0, adc_instance1, ADC_DUAL_MODE_BOTH);

	adc_enable(adc_instance0);
	adc_enable(adc_instance1);
}

void configure_dma_resource(struct dma_resource *resource, uint8_t dma_peripheral_trigger)
{
	struct dma_resource_config config;
	dma_get_config_defaults(&config);
	config.priority = DMA_PRIORITY_LEVEL_1;
	config.peripheral_trigger = dma_peripheral_trigger;
	config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	dma_allocate(resource, &config);
	resource->job_status = STATUS_OK; // we have to make sure the job status is in idle, driver don't reset to default value

}

void setup_transfer_descriptor(DmacDescriptor *descriptor, DmacDescriptor *followingDescriptior, const uint32_t source_addr, const uint32_t dest_addr)
{
	struct dma_descriptor_config descriptor_config;
	dma_descriptor_get_config_defaults(&descriptor_config);
	descriptor_config.block_action = DMA_BLOCK_ACTION_INT;
	descriptor_config.beat_size = DMA_BEAT_SIZE_HWORD;
	descriptor_config.dst_increment_enable = true;
	descriptor_config.src_increment_enable = false;
	descriptor_config.block_transfer_count = 4;
	descriptor_config.step_size = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
	descriptor_config.step_selection = DMA_STEPSEL_SRC;
	descriptor_config.source_address = (source_addr);
	descriptor_config.destination_address = (dest_addr+8); // +8 because of DstAddr = DstAddr_start + BTCNT*(BEATSIZE)
	descriptor_config.next_descriptor_address = (uint32_t)followingDescriptior;

	dma_descriptor_create(descriptor, &descriptor_config);
}

void dma_callback_transfer_done0(struct dma_resource *const resource) {
	port_pin_set_output_level(PIN_PA14, false);
	if (eeprom_data_ptr != NULL) {
		eeprom_data_ptr->adc_counter++;
	}
}

void dma_callback_transfer_done1(struct dma_resource * const resource) {
	port_pin_set_output_level(PIN_PA19, false);
}

void vAdcTask(void *pvParameters){
	port_pin_set_output_level(PIN_PA15, true);

	adc_module_t  *adc_instance0 = NULL;
	adc_module_t  *adc_instance1 = NULL;
	dma_resource_t *dma_resource0 = NULL;
	dma_resource_t *dma_resource1 = NULL;

	adc_instance0 = ((adctask_params *)pvParameters)->adc_instance0;
	adc_instance1 = ((adctask_params *)pvParameters)->adc_instance1;
	dma_resource0 = ((adctask_params *)pvParameters)->dma_resource0;
	dma_resource1 = ((adctask_params *)pvParameters)->dma_resource1;

	ulog_s("start ADC Task ...\r\n");

	adc_i2c_eeprom_t eeprom_data = {.adc_counter = 0xFFFF, .adc_tank_channel ={{0}}, .adc_temp_channel = {{0}}, .preamble = {'V', 'E'}, .sw_rev=1, .serial_no = {0}, .gain_correction=0, .offset_correction=0};
	eeprom_data_ptr = &eeprom_data;

	DmacDescriptor dma_desc_adc0_buf0 SECTION_DMAC_DESCRIPTOR;
	DmacDescriptor dma_desc_adc1_buf0 SECTION_DMAC_DESCRIPTOR;
	DmacDescriptor dma_desc_adc0_buf1 SECTION_DMAC_DESCRIPTOR;
	DmacDescriptor dma_desc_adc1_buf1 SECTION_DMAC_DESCRIPTOR;

	port_pin_set_output_level(PIN_PA15, false);

	// configure DMA for ADC0
	configure_dma_resource(dma_resource0, ADC0_DMAC_ID_RESRDY);
	setup_transfer_descriptor(&dma_desc_adc0_buf0, &dma_desc_adc0_buf1, (uint32_t ) &adc_instance0->hw->RESULT.reg, (uint32_t ) &(eeprom_data.adc_tank_channel[0]));
	setup_transfer_descriptor(&dma_desc_adc0_buf1, &dma_desc_adc0_buf0, (uint32_t ) &adc_instance0->hw->RESULT.reg, (uint32_t ) &(eeprom_data.adc_tank_channel[1]));
	dma_add_descriptor(dma_resource0, &dma_desc_adc0_buf0);
	dma_register_callback(dma_resource0, dma_callback_transfer_done0, DMA_CALLBACK_TRANSFER_DONE); //temporarily, for debug purposes.
	dma_enable_callback(dma_resource0, DMA_CALLBACK_TRANSFER_DONE);
	dma_start_transfer_job(dma_resource0);

	// configure DMA for ADC1
	configure_dma_resource(dma_resource1, ADC1_DMAC_ID_RESRDY);

	setup_transfer_descriptor(&dma_desc_adc1_buf0, &dma_desc_adc1_buf1, (uint32_t ) &adc_instance1->hw->RESULT.reg, (uint32_t ) &(eeprom_data.adc_temp_channel[0]));
	setup_transfer_descriptor(&dma_desc_adc1_buf1, &dma_desc_adc1_buf0, (uint32_t ) &adc_instance1->hw->RESULT.reg, (uint32_t ) &(eeprom_data.adc_temp_channel[1]));
	dma_add_descriptor(dma_resource1, &dma_desc_adc1_buf0);
	dma_register_callback(dma_resource1, dma_callback_transfer_done1, DMA_CALLBACK_TRANSFER_DONE); //only for slave adc, since master prio is a little bit higher and the slave will be complete last.
	dma_enable_callback(dma_resource1, DMA_CALLBACK_TRANSFER_DONE);
	dma_start_transfer_job(dma_resource1);




	// configue adc (ADC0 and ADC1)
	configure_adc(adc_instance0, adc_instance1);


	while(1) {
		port_pin_set_output_level(PIN_PA15, true);
		// idle
		bool status_busy = false;
		uint8_t sequence_state = 0;
		uint16_t result = 0;

		adc_get_sequence_status(adc_instance0, &status_busy, &sequence_state);
		if (!status_busy) {

			port_pin_set_output_level(PIN_PA15, false);
			port_pin_set_output_level(PIN_PA15, true);
			for (uint8_t k = 0; k < 2; k++) {

				ulog_s("\r\nbuffer: ");
				xlog(&k,1);

				ulog_s("\ttank sensor readings: ");
				for(uint8_t i = 0; i < 4; i++) {
					xlog_short(&eeprom_data.adc_tank_channel[k][i]);
					ulog_s(" ");
				}

				ulog_s("\ttemp sensor readings: ");
				for(uint8_t i = 0; i < 4; i++) {
					xlog_short(&eeprom_data.adc_temp_channel[k][i]);
					ulog_s(" ");
				}

				ulog_s("cnt: ");
				xlog_short(&eeprom_data.adc_counter);
			}
			ulog_s("\r\n");

			port_pin_set_output_level(PIN_PA15, false);
			port_pin_set_output_level(PIN_PA15, true);

			port_pin_set_output_level(PIN_PA14, false);
			port_pin_set_output_level(PIN_PA19, false);
			port_pin_set_output_level(PIN_PA14, true);
			port_pin_set_output_level(PIN_PA19, true);
			adc_start_conversion(adc_instance0);
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