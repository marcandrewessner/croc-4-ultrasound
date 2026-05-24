# Copyright 2025 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

# Authors:
# - Micha Wehrli <miwehrli@student.ethz.ch>

VERILATOR ?= verilator

RTL   ?= ../../..
SDHCI ?= ..

all: dat acmd12

obj_dir/V%: %.sv
	$(VERILATOR) --quiet --binary -j 0 --assert --trace --trace-structs --trace-fst --timing --autoflush -O3 -CFLAGS "-O1 -march=native" \
		-Wno-style -Wno-WIDTHEXPAND -Wno-fatal \
		--top $* \
		-f $(RTL)/../verilator/croc.f \
		$*

dat: obj_dir/Vtb_dat
	set -e; \
	for BlockSize in $$(seq 1 128); do \
	for ClkEnPeriod in 1 2 3; do \
	for UseWideBus in 0 1 ; do \
		obj_dir/Vtb_dat +verilator+quiet +UseWideBus=$$UseWideBus +BlockSize=$$BlockSize +ClkEnPeriod=$$ClkEnPeriod ; \
	done \
	done \
	done

acmd12: obj_dir/Vtb_acmd12
	set -e; \
	for Cycles in $$(seq -10 10); do \
	for ClkEnPeriod in 1 2 4; do \
	for Valid in 0 1 ; do \
	obj_dir/Vtb_acmd12 +verilator+quiet \
		+CyclesThatDriverCommandArrivesBeforeCMD12=$$Cycles \
		+IsFirstResponseValid=$$Valid \
		+ClkEnPeriod=$$ClkEnPeriod ; \
	done \
	done \
	done

.PHONY: acmd12 dat all
