//
// Created by Felix Hübner on 2019-05-20.
//

#ifndef SAMC_FREERTOS_ADC_TASK_H
#define SAMC_FREERTOS_ADC_TASK_H
#include "asf.h"
#include "adc_sam_l_c/adc_feature.h"
#include "dma.h"
#include "i2c_slave.h"
#include "i2c_vitual_eeprom.h"
#include "adc_methods.h"


/******** Typedef ********/
typedef struct {
	adc_module_t *adc_instance0;
	adc_module_t *adc_instance1;
	i2c_module_t *i2c_instance;
	dma_resource_t *dma_resource0;
	dma_resource_t *dma_resource1;
} adctask_params;

/******** Prototypes ********/
TaskHandle_t vCreateAdcTask(adctask_params *params);

#endif //SAMC_FREERTOS_ADC_TASK_H
