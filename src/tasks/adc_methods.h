//
// Created by Felix Hübner on 2019-08-28.
//

#ifndef SAMC_FREERTOS_ADC_METHODS_H
#define SAMC_FREERTOS_ADC_METHODS_H

typedef struct adc_module adc_module_t;
typedef struct dma_resource dma_resource_t;

void configure_adc_dma(dma_resource_t* dma_resource[2], adc_module_t* adc_instances[2], DmacDescriptor dma_desc[4], uint32_t dest_ptr_adr[4]);

void adc_trigger_new_conv(void);
bool is_adc_busy(void);

#endif //SAMC_FREERTOS_ADC_METHODS_H
