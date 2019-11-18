//
// Created by Felix Hübner on 2019-08-28.
//

#include "asf.h"
#include "adc_methods.h"
#include <adc.h>
#include <adc_sam_l_c/adc_feature.h>
#include <dma.h>
#include <adc_callback.h>
#include <log.h>
#include "i2c_vitual_eeprom.h"

/******** Defines ********/
#define ADC_MAX_RESOLUTION		4096
#define ADC_REF_VOLTAGE			2.0625f
#define ADC_LOW_REF_VOLTAGE		0.21f
#define ADC_HIGH_REF_VOLTAGE	1.198f
#define ADC_GAIN_CORR_MIDDLE	2048
#define STR_BUFFER_LEN			50

/******** Global Variables ********/
adc_module_t *master_adc_module = NULL;


/******** Internal Prototypes ********/
void dma_callback_transfer_done0(struct dma_resource *const resource);
void dma_callback_transfer_done1(struct dma_resource *const resource);
void configure_adc(adc_module_t *adc_instance0, adc_module_t *adc_instance1, bool use_correction, const uint16_t gain_corrections[2], const int16_t offset_corrections[2]);
void configure_adc_dma_resource(struct dma_resource *resource, uint8_t dma_peripheral_trigger);
void setup_transfer_descriptor(DmacDescriptor *descriptor, DmacDescriptor *followingDescriptior, uint32_t source_addr, uint32_t dest_addr, uint8_t length);
void configure_adc_for_calibration(adc_module_t *adc_instance0, adc_module_t *adc_instance1);
void adc_calculate_correction_values(uint16_t high_voltage, uint16_t low_voltage, uint16_t *gain_corr, int16_t *offset_corr);

/**
 * Configure both ADCs to read the reference resistors (available to both ADCs)
 * @param adc_instance0 pointer to the master adc instance
 * @param adc_instance1 pointer to the slave adc instance
 */
void configure_adc_for_calibration(adc_module_t *adc_instance0, adc_module_t *adc_instance1) {
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);


	//configure ADC0
	config_adc.resolution		= ADC_RESOLUTION_CUSTOM;
	config_adc.clock_source		= GCLK_GENERATOR_2;
	config_adc.clock_prescaler	= ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference		= ADC_REFERENCE_INTVCC0;
	config_adc.negative_input	= ADC_NEGATIVE_INPUT_GND;
	config_adc.positive_input	= ADC_POSITIVE_INPUT_PIN8;
	config_adc.freerunning		= false;
	config_adc.left_adjust		= false;
	config_adc.run_in_standby	= true;
	config_adc.window.window_mode = ADC_WINDOW_MODE_DISABLE;
	config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_64;
	config_adc.divide_result = ADC_DIVIDE_RESULT_16;
	config_adc.sample_length = 10;
	config_adc.correction.correction_enable = false;
	if (adc_init(adc_instance0, ADC0, &config_adc) != STATUS_OK) {
		ulog_s("ERROR: while setup ADC1!\r\n");
	}

	config_adc.positive_input	= ADC_POSITIVE_INPUT_PIN11;
	if (adc_init(adc_instance1, ADC1, &config_adc) != STATUS_OK) {
		ulog_s("ERROR: while setup ADC1!\r\n");
	}

	adc_set_master_slave_mode(adc_instance0, adc_instance1, ADC_DUAL_MODE_BOTH);

	if (adc_enable(adc_instance0) != STATUS_OK) {
		ulog_s("ERROR: while start ADC0!\r\n");
	}
	if (adc_enable(adc_instance1) != STATUS_OK) {
		ulog_s("ERROR: while start ADC1!\r\n");
	}
}

/******** Methods ********/
/**
 * configures both adc instrances
 * @param adc_instance0 pointer to the master adc instance
 * @param adc_instance1 pointer to the slave adc instance
 * @param use_correction true if the calibration values given in gain_correction and offset_correction should be used.
 * @param gain_corrections array that contain the gain correction values
 * @param offset_correctionsarray that contain the offset correction values
 */
void configure_adc(adc_module_t *adc_instance0, adc_module_t *adc_instance1, bool use_correction, const uint16_t gain_corrections[2], const int16_t offset_corrections[2]) {
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);


	//configure ADC0
	config_adc.resolution		= ADC_RESOLUTION_CUSTOM;
	config_adc.clock_source		= GCLK_GENERATOR_2;
	config_adc.clock_prescaler	= ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference		= ADC_REFERENCE_INTVCC0;
	config_adc.negative_input	= ADC_NEGATIVE_INPUT_GND;
	config_adc.freerunning		= false;
	config_adc.left_adjust		= false;
	config_adc.run_in_standby	= true;
	config_adc.positive_input_sequence_mask_enable = 1 << ADC_POSITIVE_INPUT_PIN4 | 1 << ADC_POSITIVE_INPUT_PIN5 | 1 << ADC_POSITIVE_INPUT_PIN6 | 1 << ADC_POSITIVE_INPUT_PIN7 | 1 << ADC_POSITIVE_INPUT_PIN1; //PA04-PA07, Vclap is at PA03 (AIN(1)
	config_adc.window.window_mode = ADC_WINDOW_MODE_DISABLE;
	config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_64;
	config_adc.divide_result = ADC_DIVIDE_RESULT_16;
	config_adc.sample_length = 10;
	config_adc.correction.correction_enable = use_correction;
	config_adc.correction.gain_correction = gain_corrections[0];
	config_adc.correction.offset_correction = offset_corrections[0];

	if (adc_init(adc_instance0, ADC0, &config_adc) != STATUS_OK) {
		ulog_s("ERROR: while setup ADC1!\r\n");
	}
	adc_enable_positive_input_sequence(adc_instance0, config_adc.positive_input_sequence_mask_enable);

	//configure ADC1
	config_adc.positive_input_sequence_mask_enable = 1 << ADC_POSITIVE_INPUT_PIN2 | 1 << ADC_POSITIVE_INPUT_PIN3 | 1 << ADC_POSITIVE_INPUT_PIN4 | 1 << ADC_POSITIVE_INPUT_PIN5; //PB02-PB03,PB08-PB09
	config_adc.correction.gain_correction = gain_corrections[1];
	config_adc.correction.offset_correction = offset_corrections[1];
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


/**
 * configures the dma resource of the adc conversion result transfer
 * @param resource dma-resource that should be configured
 * @param dma_peripheral_trigger DMA trigger source
 */
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


/**
 * configures a transfer of the dma controller (transfer 4x 16Bit values and increment the write addresses after each write in the block)
 * @param descriptor dma descriptor to configure
 * @param followingDescriptior pointer to a following descriptor if there is any, otherwise NULL
 * @param source_addr address to read from
 * @param dest_addr address to write to
 * @param length number of transfers this descriptior has to do
 */
void setup_transfer_descriptor(DmacDescriptor *descriptor, DmacDescriptor *followingDescriptior, const uint32_t source_addr, const uint32_t dest_addr, uint8_t length) {
	struct dma_descriptor_config descriptor_config;
	dma_descriptor_get_config_defaults(&descriptor_config);
	descriptor_config.block_action = DMA_BLOCK_ACTION_INT;
	descriptor_config.beat_size = DMA_BEAT_SIZE_HWORD;
	descriptor_config.dst_increment_enable = true;
	descriptor_config.src_increment_enable = false;
	descriptor_config.block_transfer_count = length;
	descriptor_config.step_size = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
	descriptor_config.step_selection = DMA_STEPSEL_SRC;
	descriptor_config.source_address = (source_addr);
	descriptor_config.destination_address = (dest_addr + (2*length)); // +8 because of DstAddr = DstAddr_start + BTCNT*(BEATSIZE)
	descriptor_config.next_descriptor_address = (uint32_t)followingDescriptior;
	dma_descriptor_create(descriptor, &descriptor_config);
}


/**
 * configure the hole dual adc and dma struture, both adcs work as a unit to read 8 channels and use two dma resources to transfer the results to the eeprom array
 * @param dma_resource both used dma resources
 * @param adc_instances both adc instances
 * @param dma_desc 4 descriptors, two for each dma resource, because we use double buffering for the write location
 * @param dest_ptr_adr 4 adr, two for each dma resource, because we use double buffering for the write location
 * @param use_correction true if the calibration values given in gain_correction and offset_correction should be used.
 * @param gain_corrections array that contain the gain correction values
 * @param offset_correctionsarray that contain the offset correction values
 */
void configure_adc_dma(dma_resource_t* dma_resource[2], adc_module_t* adc_instances[2], DmacDescriptor dma_desc[4], uint32_t dest_ptr_adr[4], bool use_correction, const uint16_t gain_corrections[2], const int16_t offset_corrections[2]) {
	configASSERT(adc_instances);
	configASSERT(adc_instances[0]);
	configASSERT(adc_instances[1]);
	configASSERT(dma_resource);
	configASSERT(dma_resource[0]);
	configASSERT(dma_resource[1]);
	configASSERT(dma_desc);
	configASSERT(dest_ptr_adr)

	// configue adc (ADC0 and ADC1)
	configure_adc(adc_instances[0], adc_instances[1], use_correction, gain_corrections, offset_corrections);

	// configure DMA for ADC0
	configure_adc_dma_resource(dma_resource[0], ADC0_DMAC_ID_RESRDY);
	setup_transfer_descriptor(&dma_desc[0], &dma_desc[1], (uint32_t) &adc_instances[0]->hw->RESULT.reg, dest_ptr_adr[0],5);
	setup_transfer_descriptor(&dma_desc[1], &dma_desc[0], (uint32_t) &adc_instances[0]->hw->RESULT.reg, dest_ptr_adr[1],5);
	dma_add_descriptor(dma_resource[0], &dma_desc[0]);
	dma_register_callback(dma_resource[0], dma_callback_transfer_done0, DMA_CALLBACK_TRANSFER_DONE); //temporarily, for debug purposes.
	dma_enable_callback(dma_resource[0], DMA_CALLBACK_TRANSFER_DONE);
	if (dma_start_transfer_job(dma_resource[0]) != STATUS_OK) {
		ulog_s("ERROR: Start of ADC0 DMA failed!\r\n");
	}

	// configure DMA for ADC1
	configure_adc_dma_resource(dma_resource[1], ADC1_DMAC_ID_RESRDY);
	setup_transfer_descriptor(&dma_desc[2], &dma_desc[3], (uint32_t) &adc_instances[1]->hw->RESULT.reg, dest_ptr_adr[2],4);
	setup_transfer_descriptor(&dma_desc[3], &dma_desc[2], (uint32_t) &adc_instances[1]->hw->RESULT.reg, dest_ptr_adr[3],4);
	dma_add_descriptor(dma_resource[1], &dma_desc[2]);
	dma_register_callback(dma_resource[1], dma_callback_transfer_done1, DMA_CALLBACK_TRANSFER_DONE); //only for slave adc, since master prio is a little bit higher and the slave will be complete last.
	dma_enable_callback(dma_resource[1], DMA_CALLBACK_TRANSFER_DONE);
	if (dma_start_transfer_job(dma_resource[1]) != STATUS_OK) {
		ulog_s("ERROR: Start of ADC1 DMA failed!\r\n");
	}
}


/**
 * do the linear calibration (based on this: http://ww1.microchip.com/downloads/en/DeviceDoc/90003185A.pdf)
 * @param high_voltage high reference voltage
 * @param low_voltage low reference voltage
 * @param gain_corr pointer to store the gain_correction value
 * @param offset_corr pointer to store the offset_correection value
 */
void adc_calculate_correction_values(uint16_t high_voltage, uint16_t low_voltage, uint16_t *gain_corr, int16_t *offset_corr) {
	uint16_t ideal_voltage[2] = {0};
	ideal_voltage[0] = (uint16_t)((ADC_LOW_REF_VOLTAGE * ADC_MAX_RESOLUTION) / ADC_REF_VOLTAGE);
	ideal_voltage[1] = (uint16_t)((ADC_HIGH_REF_VOLTAGE * ADC_MAX_RESOLUTION) / ADC_REF_VOLTAGE);

	float gain_error = ((float) high_voltage - (float)low_voltage) / ((float) ideal_voltage[1] - (float) ideal_voltage[0]);
	*gain_corr = (uint16_t) (ADC_GAIN_CORR_MIDDLE / gain_error);
	*offset_corr = (int16_t) (low_voltage-(gain_error * ideal_voltage[0]));
}


/**
 * do a calibration of both ADC converters, will read two known voltages and calculate a linear correction
 * @param adc_instances array with both adc instances
 * @param gain_corrections array to store the gain correction values
 * @param offset_corrections array to store the offset correction values
 */
void adc_perform_calibration(adc_module_t* adc_instances[2], uint16_t gain_corrections[2], int16_t offset_corrections[2]) {
	uint16_t high_voltage[2] = {0};
	uint16_t low_voltage[2] = {0};

	ulog_s("calculate calibration and apply to ADCs\r\n");
	configure_adc_for_calibration(adc_instances[0], adc_instances[1]);
	adc_start_conversion(adc_instances[0]);

	//wait until both adc are ready
	while(adc_read(adc_instances[0], &low_voltage[0]) == STATUS_BUSY){};
	while(adc_read(adc_instances[1], &high_voltage[1]) == STATUS_BUSY){};

	//switch channels
	adc_set_positive_input(adc_instances[0], ADC_POSITIVE_INPUT_PIN9);
	adc_set_positive_input(adc_instances[1], ADC_POSITIVE_INPUT_PIN10);
	adc_start_conversion(adc_instances[0]);

	//wait until both adc are ready
	while(adc_read(adc_instances[0], &high_voltage[0]) == STATUS_BUSY){};
	while(adc_read(adc_instances[1], &low_voltage[1]) == STATUS_BUSY){};

	//calculate correction values
	adc_calculate_correction_values(high_voltage[0], low_voltage[0], &gain_corrections[0], &offset_corrections[0]);
	adc_calculate_correction_values(high_voltage[1], low_voltage[1], &gain_corrections[1], &offset_corrections[1]);

	//disable (calibration setup) ADC
	adc_disable(adc_instances[0]);
	adc_disable(adc_instances[1]);
}


/**
 * callback that will be called if the dma transfer is done for dma resource 0
 * @param resource dma resource
 */
void dma_callback_transfer_done0(struct dma_resource *const resource) {
}


/**
 * callback that will be called if the dma transfer is done for dma resource 1, this will finished last, so this will increment the counter in the eeprom
 * @param resource dma resource
 */
void dma_callback_transfer_done1(struct dma_resource * const resource) {
	inc_adc_data_counter();
}


/**
 * method that will initiate a new adc conversion
 */
void adc_trigger_new_conv(void) {
	adc_start_conversion(master_adc_module);
}


/******** Getter/Setter ********/
/**
 * will return if the adc is currently busy or idle
 * @return true if adc is busy otherwise false
 */
bool is_adc_busy(void) {
	bool status_busy = false;
	uint8_t sequence_state = 0;
	adc_get_sequence_status(master_adc_module, &status_busy, &sequence_state);
	return status_busy;
}