#!/bin/bash

# NOTE THIS IS RUN INSIDE THE OPENTITAN DOCKER CONTAINER

# Use this bash script to generate
# docs, register, c ....
# from the register configuration
# note: it uses regtool from opentitan

cd /home/dev
echo "GENERATING ADC REGISTERS"

REGTOOL=opentitan/util/regtool.py

REGDEFINITION=src/rtl/adc_acquisition/reg/adc_acquisition_reg_definition.hjson

# Generate the RTL
"$REGTOOL" src/rtl/adc_acquisition/reg/adc_acquisition_reg_definition.hjson -r --outdir src/rtl/adc_acquisition/reg
# Generate the Markdown
