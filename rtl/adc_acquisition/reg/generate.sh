#!/bin/bash

# Note we use peakrdl for generation
echo "Generating ADC Acq Registers & Docs"

peakrdl regblock -o . adc_acquisition_reg_definition.rdl --cpuif obi-flat