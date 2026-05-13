#!/bin/bash

# Note we use peakrdl for generation
echo "Generating ADC Acq Registers & Docs"

peakrdl regblock -o . adc_acquisition_reg_definition.rdl --cpuif obi-flat
peakrdl html -o doc adc_acquisition_reg_definition.rdl
peakrdl c-header -o ../../../sw/lib/inc/adc_acquisition_reg.h --std gnu99 adc_acquisition_reg_definition.rdl 