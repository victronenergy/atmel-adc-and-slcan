//
// Created by Felix Hübner on 2019-08-28.
//

#include "asf.h"
#include <samc21_slcan_adc.h>
#include "adc_methods.h"
#include <adc.h>
#include <adc_sam_l_c/adc_feature.h>
#include <dma.h>
#include <adc_callback.h>
#include <samc21_slcan_adc.h>
#include <log.h>
#include "i2c_vitual_eeprom.h"

void dma_callback_transfer_done0(struct dma_resource *const resource);
void dma_callback_transfer_done1(struct dma_resource *const resource);
void configure_adc(adc_module_t *adc_instance0, adc_module_t *adc_instance1);
void configure_adc_dma_resource(struct dma_resource *resource, uint8_t dma_peripheral_trigger);
void setup_transfer_descriptor(DmacDescriptor *descriptor, DmacDescriptor *followingDescriptior, uint32_t source_addr, uint32_t dest_addr);

adc_module_t *master_adc_module = NULL;

void configure_adc(adc_module_t *adc_instance0, adc_module_t *adc_instance1) {
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);


	//configure ADC0
	config_adc.resolution		= ADC_RESOLUTION_CUSTOM;
	config_adc.clock_source		= GCLK_GENERATOR_2;
	config_adc.clock_prescaler	= ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference		= ADC_REFERENCE_INTVCC2;
//	config_adc.reference		= ADC_REFERENCE_AREFA;
	config_adc.negative_input	= ADC_NEGATIVE_INPUT_GND;
	config_adc.freerunning		= false;
	config_adc.left_adjust		= false;
	config_adc.run_in_standby	= true;
	config_adc.positive_input_sequence_mask_enable = 1 << ADC_POSITIVE_INPUT_PIN4 | 1 << ADC_POSITIVE_INPUT_PIN5 | 1 << ADC_POSITIVE_INPUT_PIN6 | 1 << ADC_POSITIVE_INPUT_PIN7; //PA04-PA07
	config_adc.window.window_mode = ADC_WINDOW_MODE_DISABLE;
	config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_64;
	config_adc.divide_result = ADC_DIVIDE_RESULT_16;
	config_adc.sample_length = 10;

	if (adc_init(adc_instance0, ADC0, &config_adc) != STATUS_OK) {
		ulog_s("ERROR: while setup ADC1!\r\n");
	}
	adc_enable_positive_input_sequence(adc_instance0, config_adc.positive_input_sequence_mask_enable);

	//configure ADC1
	config_adc.positive_input_sequence_mask_enable = 1 << ADC_POSITIVE_INPUT_PIN2 | 1 << ADC_POSITIVE_INPUT_PIN3 | 1 << ADC_POSITIVE_INPUT_PIN4 | 1 << ADC_POSITIVE_INPUT_PIN5; //PB02-PB03,PB08-PB09
	if (adc_init(adc_instance1, ADC1, &config_adc) != STATUS_OK) {
		ulog_s("ERROR: while setup ADC1!\r\n");
	}
	adc_enable_positive_input_sequence(adc_instance1, config_adc.positive_input_sequence_mask_enable);
	adc_set_master_slave_mode(adc_instance0, adc_instance1, ADC_DUAL_MODE_BOTH);

	master_adc_module = adc_instance0;

	if (adc_enable(adc_instance0) != STATUS_OK) {
		ulog_s("ERROR: while start ADC0!\r\n");
	}
	if (adc_enable(adc_instance1) != STATUS_OK) {
		ulog_s("ERROR: while start ADC1!\r\n");
	}
}

void configure_adc_dma_resource(struct dma_resource *resource, uint8_t dma_peripheral_trigger) {
	struct dma_resource_config config;
	dma_get_config_defaults(&config);
	config.priority = DMA_PRIORITY_LEVEL_0;
	config.peripheral_trigger = dma_peripheral_trigger;
	config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	if (dma_allocate(resource, &config) != STATUS_OK) {
		ulog_s("ERROR: while setup DMA for ADC!\r\n");
	}
	resource->job_status = STATUS_OK; // we have to make sure the job status is in idle, driver don't reset to default value

}

void setup_transfer_descriptor(DmacDescriptor *descriptor, DmacDescriptor *followingDescriptior, const uint32_t source_addr, const uint32_t dest_addr) {
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


void configure_adc_dma(dma_resource_t* dma_resource[2], adc_module_t* adc_instances[2], DmacDescriptor dma_desc[4], uint32_t dest_ptr_adr[4]) {

	configASSERT(adc_instances);
	configASSERT(adc_instances[0]);
	configASSERT(adc_instances[1]);
	configASSERT(dma_resource);
	configASSERT(dma_resource[0]);
	configASSERT(dma_resource[1]);
	configASSERT(dma_desc);
	configASSERT(dest_ptr_adr)

	ulog_s("point 1\r\n");

	// configue adc (ADC0 and ADC1)
	configure_adc(adc_instances[0], adc_instances[1]);

	// configure DMA for ADC0
	configure_adc_dma_resource(dma_resource[0], ADC0_DMAC_ID_RESRDY);
	setup_transfer_descriptor(&dma_desc[0], &dma_desc[1], (uint32_t) &adc_instances[0]->hw->RESULT.reg, dest_ptr_adr[0]);
	setup_transfer_descriptor(&dma_desc[1], &dma_desc[0], (uint32_t) &adc_instances[0]->hw->RESULT.reg, dest_ptr_adr[1]);
	dma_add_descriptor(dma_resource[0], &dma_desc[0]);
	dma_register_callback(dma_resource[0], dma_callback_transfer_done0, DMA_CALLBACK_TRANSFER_DONE); //temporarily, for debug purposes.
	dma_enable_callback(dma_resource[0], DMA_CALLBACK_TRANSFER_DONE);
	if (dma_start_transfer_job(dma_resource[0]) != STATUS_OK) {
		ulog_s("ERROR: Start of ADC0 DMA failed!\r\n");
	}

	// configure DMA for ADC1
	configure_adc_dma_resource(dma_resource[1], ADC1_DMAC_ID_RESRDY);
	setup_transfer_descriptor(&dma_desc[2], &dma_desc[3], (uint32_t) &adc_instances[1]->hw->RESULT.reg, dest_ptr_adr[2]);
	setup_transfer_descriptor(&dma_desc[3], &dma_desc[2], (uint32_t) &adc_instances[1]->hw->RESULT.reg, dest_ptr_adr[3]);
	dma_add_descriptor(dma_resource[1], &dma_desc[2]);
	dma_register_callback(dma_resource[1], dma_callback_transfer_done1, DMA_CALLBACK_TRANSFER_DONE); //only for slave adc, since master prio is a little bit higher and the slave will be complete last.
	dma_enable_callback(dma_resource[1], DMA_CALLBACK_TRANSFER_DONE);
	if (dma_start_transfer_job(dma_resource[1]) != STATUS_OK) {
		ulog_s("ERROR: Start of ADC1 DMA failed!\r\n");
	}
}


void dma_callback_transfer_done0(struct dma_resource *const resource) {
	port_pin_set_output_level(PIN_PA14, false);
}

void dma_callback_transfer_done1(struct dma_resource * const resource) {
	port_pin_set_output_level(PIN_PA19, false);
	inc_adc_data_counter();
}

void adc_trigger_new_conv(void) {
	adc_start_conversion(master_adc_module);
}

bool is_adc_busy(void) {
	bool status_busy = false;
	uint8_t sequence_state = 0;
	adc_get_sequence_status(master_adc_module, &status_busy, &sequence_state);
	return status_busy;
}