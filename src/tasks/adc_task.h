//
// Created by Felix Hübner on 2019-05-20.
//

#ifndef SAMC_FREERTOS_ADC_TASK_H
#define SAMC_FREERTOS_ADC_TASK_H
#include "asf.h"
#include "adc_sam_l_c/adc_feature.h"
#include "dma.h"

typedef struct adc_module adc_module_t;
typedef struct dma_resource dma_resource_t;

typedef struct {
	adc_module_t *adc_instance0;
	adc_module_t *adc_instance1;
	dma_resource_t *dma_resource0;
	dma_resource_t *dma_resource1;
} adctask_params;

TaskHandle_t vCreateAdcTask(adctask_params *params);

#endif //SAMC_FREERTOS_ADC_TASK_H
