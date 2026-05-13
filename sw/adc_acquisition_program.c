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

#define DATA_BANK_ADDR  0x10000800
#define ADC_BANK_0_ADDR 0x10001000
#define ADC_BANK_1_ADDR 0x10001800

volatile int adc_full;

void croc_interrupt_handler(uint32_t cause) {
  set_global_irq_enable(0);
  adc_full = 1;
}

int main() {
  adc_full = 0;
  uart_init();

  //set_interrupt_enable(1, 16+4);
  //set_global_irq_enable(1);

  
  // Wait for the interrupt to fire
  // then dump the acquired words
  for(uint32_t i=0; i<10000; i++){
    if(!adc_full)
      continue;
    printf("ADC INTERRUPT RECEIVED\n");
    break;
  }

  printf("BEGIN DUMP\n");
  for(uint32_t i=0; i<20; i++){
    uint32_t data = *reg32(ADC_BANK_0_ADDR, i);
    uint32_t data0 = data & 0x0000FFFF;
    uint32_t data1 = (data & 0xFFFF0000) >> 16;
    printf("0x%x / 0x%x \n", data0, data1);
  }
  printf("END DUMP\n");

  uart_write_flush();
  return 0;
}
