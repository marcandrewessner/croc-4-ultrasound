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

#define SDCARD_BASE_ADDR 0x10010000


int main() {
  uart_init();

  printf("Hello from SDCard PG\n");
  
  uart_write_flush();
  return 0;
}
