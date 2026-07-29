# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# GDB batch script: load an ELF into Croc SRAM over JTAG/OpenOCD and kick
# off execution. Invoked by run_sw.sh, but can be run standalone:
#   riscv64-unknown-elf-gdb -batch -x load_run.gdb <path/to/prog.elf>
# (requires openocd already running with openocd.genesys2.tcl)

set pagination off
set confirm off

target extended-remote localhost:3333

load

# The bootrom sits in WFI after reset, waiting for the debugger to write a
# program into SRAM and wake it via the CLINT (see rtl/bootrom/README.md).
# Point BOOTADDR at the ELF's entry point, then set msip to wake the core.
set *(unsigned int*)0x03000000 = (unsigned int)&_start
set *(unsigned int*)0x02040000 = 1

# Use OpenOCD's own (asynchronous) resume via 'monitor' instead of gdb's
# 'continue', which would block waiting for a stop reply that never comes
# since the target free-runs.
monitor resume
detach
quit
