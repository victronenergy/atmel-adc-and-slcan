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
 * configure the hole dual adc and dma struture, both adcs work as a unit to read 8 channels and use two dma resources to transfer the results to the eeprom array
 * @param dma_resource both used dma resources
 * @param adc_instances both adc instances
 * @param dma_desc 4 descriptors, two for each dma resource, because we use double buffering for the write location
 * @param dest_ptr_adr 4 adr, two for each dma resource, because we use double buffering for the write location
 */
void configure_adc_dma(dma_resource_t* dma_resource[2], adc_module_t* adc_instances[2], DmacDescriptor dma_desc[4], uint32_t dest_ptr_adr[4]);

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
