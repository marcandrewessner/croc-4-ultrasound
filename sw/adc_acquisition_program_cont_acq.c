// Copyright (c) 2024 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Authors:
// - Marc-André Wessner

#include <stdint.h>

#include "uart.h"
#include "print.h"
#include "config.h"
#include "util.h"
#include "adc_acquisition.h"

#define DATA_BANK_ADDR  0x10000800
#define ADC_BANK_0_ADDR 0x10001000
#define ADC_BANK_1_ADDR 0x10001800

static volatile int adc_frame0_full;
static volatile int adc_frame1_full;

// Interrupt handler
void croc_interrupt_handler(uint32_t cause) {
  if(cause == ADC_ACQ_INTERRUPT){
    if(ADC_ACQ->STATUS & 1<<ADC_ACQ_STATUS_F0_FULL_BIT){
      adc_frame0_full = 1;
      ADC_ACQ->CNTRL = 1<<ADC_ACQ_CTRL_CLR_F0_FULL_BIT;
    }
    if(ADC_ACQ->STATUS & 1<<ADC_ACQ_STATUS_F1_FULL_BIT){
      adc_frame1_full = 1;
      ADC_ACQ->CNTRL = 1<<ADC_ACQ_CTRL_CLR_F1_FULL_BIT;
    }
  }
}


int main() {
  uart_init();
  // Enable the interrupt
  set_interrupt_enable(1, ADC_ACQ_INTERRUPT);
  set_global_irq_enable(1);

  adc_frame0_full = 0;
  adc_frame1_full = 0;

  // Set into idle
  ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;
  // Setup the frame
  ADC_ACQ->F0_START_ADDR = ADC_BANK_0_ADDR;
  ADC_ACQ->F0_END_ADDR   = ADC_BANK_0_ADDR+0x800;
  ADC_ACQ->F1_START_ADDR = ADC_BANK_1_ADDR;
  ADC_ACQ->F1_END_ADDR   = ADC_BANK_1_ADDR+0x800;
  // Reset write head and clear full interrupts
  ADC_ACQ->CNTRL         = 0b111;
  // Start the ADC ACQ
  ADC_ACQ->CONF = ADC_ACQ_MODE_CONTINUOUS_ACQ_F0_F1;

  for(uint32_t j=0; j<4; j++){
    // Wait for full buffer
    printf("WB\n");
    while(!(adc_frame0_full | adc_frame1_full));
    // Depending on full frame
    if(adc_frame0_full & 0){
      adc_frame0_full = 0;
      printf("BD F0\n");
      /*
      for(uint32_t i=0; i<1; i++){
        uint32_t data = *reg32(ADC_BANK_0_ADDR, 4*i);
        uint32_t data0 = data & 0x0000FFFF;
        uint32_t data1 = (data & 0xFFFF0000) >> 16;
        printf("0x%x / 0x%x \n", data0, data1);
      }
      printf("END DUMP\n");
      */
    }
    else if(adc_frame1_full) {
      adc_frame1_full = 0;
      printf("BD F1\n");
      /*
      for(uint32_t i=0; i<1; i++){
        uint32_t data = *reg32(ADC_BANK_1_ADDR, 4*i);
        uint32_t data0 = data & 0x0000FFFF;
        uint32_t data1 = (data & 0xFFFF0000) >> 16;
        printf("0x%x / 0x%x \n", data0, data1);
      }
      printf("END DUMP\n");
      */
    }
  }

  ADC_ACQ->CONF = ADC_ACQ_MODE_IDLE;

  uart_write_flush();
  return 0;
}
