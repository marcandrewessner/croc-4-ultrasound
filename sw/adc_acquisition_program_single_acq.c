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

static volatile int adc_full;

// Interrupt handler
void croc_interrupt_handler(uint32_t cause) {
  if(cause == ADC_ACQ_INTERRUPT){
    ADC_ACQ->CNTRL = (
      1<<ADC_ACQ_CTRL_CLR_F0_FULL_BIT |
      1<<ADC_ACQ_CTRL_CLR_F1_FULL_BIT
    );
    adc_full=1;
  }
}


int main() {
  uart_init();
  // Enable the interrupt
  set_interrupt_enable(1, ADC_ACQ_INTERRUPT);
  set_global_irq_enable(1);

  adc_full = 0;

  uint32_t n_words = 10;

  // Setup the frame
  ADC_ACQ->F0_START_ADDR = ADC_BANK_0_ADDR;
  ADC_ACQ->F0_END_ADDR   = ADC_BANK_0_ADDR+n_words*4;
  // Reset write head and clear full interrupts
  ADC_ACQ->CNTRL         = 0b111;
  // Start the ADC ACQ
  ADC_ACQ->CONF = ADC_ACQ_MODE_SINGLE_ACQ_F0;

  // Poll for full
  printf("POLL FOR F0 FULL\n");
  //for(volatile uint32_t i=0; i<10000; i++);
  //while(!(ADC_ACQ->STATUS & 1<<ADC_ACQ_STATUS_F0_FULL_BIT));
  while(!adc_full);
  //Clear interrupt and set idle mode
  //ADC_ACQ->STATUS = ADC_ACQ_MODE_IDLE | 1<<ADC_ACQ_STATUS_F0_FULL_BIT;

  printf("BEGIN DUMP\n");
  for(uint32_t i=0; i<n_words; i++){
    uint32_t data = *reg32(ADC_BANK_0_ADDR, 4*i);
    uint32_t data0 = data & 0x0000FFFF;
    uint32_t data1 = (data & 0xFFFF0000) >> 16;
    printf("0x%x / 0x%x \n", data0, data1);
  }
  printf("END DUMP\n");

  uart_write_flush();
  return 0;
}
