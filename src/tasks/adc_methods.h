//
// Created by Felix Hübner on 2019-08-28.
//

#ifndef SAMC_FREERTOS_ADC_METHODS_H
#define SAMC_FREERTOS_ADC_METHODS_H

/******** Typedef ********/
typedef struct adc_module adc_module_t;
typedef struct dma_resource dma_resource_t;

/******** Prototypes ********/
/**
 * do a calibration of both ADC converters, will read two known voltages and calculate a linear correction
 * @param adc_instances array with both adc instances
 * @param gain_corrections array to store the gain correction values
 * @param offset_corrections array to store the offset correction values
 */
void adc_perform_calibration(adc_module_t* adc_instances[2], uint16_t gain_corrections[2], int16_t offset_corrections[2]);

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
void configure_adc_dma(dma_resource_t* dma_resource[2], adc_module_t* adc_instances[2], DmacDescriptor dma_desc[4], uint32_t dest_ptr_adr[4], bool use_correction, const uint16_t gain_corrections[2], const int16_t offset_corrections[2]);

/**
 * method that will initiate a new adc conversion
 */
void adc_trigger_new_conv(void);

/**
 * will return if the adc is currently busy or idle
 * @return true if adc is busy otherwise false
 */
bool is_adc_busy(void);

#endif //SAMC_FREERTOS_ADC_METHODS_H
