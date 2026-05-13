
#pragma once

#include "util.h"

#include "adc_acquisition_reg.h"

#define ADC_ACQ_BASE 0x0300C000UL

// export an accessable register
#define ADC_ACQ ((volatile adc_acquisition_reg_t*)ADC_ACQ_BASE)

// Setup the modes
#define ADC_ACQ_MODE_IDLE                 0x00
#define ADC_ACQ_MODE_SINGLE_ACQ_F0        0x04
#define ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1 0x10

// Setup the interupt flags
#define ADC_ACQ_STATUS_F0_FULL_BIT 9
#define ADC_ACQ_STATUS_F1_FULL_BIT 10