# Copyright 2025 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

# Authors:
# - Micha Wehrli <miwehrli@student.ethz.ch>
# - Axel Vanoni <axvanoni@student.ethz.ch>

$(SDHCI_ROOT)/target/sim/model/sd_crc_7.v:
	curl -o $@ https://raw.githubusercontent.com/fabriziotappero/ip-cores/refs/heads/communication_controller_wishbone_sd_card_controller/rtl/verilog/sd_crc_7.v
$(SDHCI_ROOT)/target/sim/model/sd_crc_16.v:
	curl -o $@ https://raw.githubusercontent.com/fabriziotappero/ip-cores/refs/heads/communication_controller_wishbone_sd_card_controller/rtl/verilog/sd_crc_16.v
$(SDHCI_ROOT)/target/sim/model/sdModel.v:
	curl -o $@ https://gist.githubusercontent.com/micha4w/38fad58c1cea3157f204709d4eca227e/raw/6bd185335ec809baef707e8ec3cdde7fd5bcb6fa/sdModel.v

VLOG_ARGS ?= -timescale=1ns/1ps

$(SDHCI_ROOT)/target/sim/vsim/compile.sdhci.tcl: $(SDHCI_ROOT)/Bender.yml $(SDHCI_ROOT)/Bender.lock
	$(BENDER) script vsim -t simulation -t test -t vsim --vlog-arg="$(VLOG_ARGS)" > $@


sdhci-sim-model-all: $(SDHCI_ROOT)/target/sim/model/sd_crc_7.v
sdhci-sim-model-all: $(SDHCI_ROOT)/target/sim/model/sd_crc_16.v
sdhci-sim-model-all: $(SDHCI_ROOT)/target/sim/model/sdModel.v

sdhci-sim-all: sdhci-sim-model-all
sdhci-sim-all: $(SDHCI_ROOT)/target/sim/vsim/compile.sdhci.tcl

sdhci-sim-vsim-clean:
	rm -fr $(SDHCI_ROOT)/target/sim/vsim/work
	rm -fr $(SDHCI_ROOT)/target/sim/vsim/vsim.wlf
	rm -fr $(SDHCI_ROOT)/target/sim/vsim/tb_acmd12.vcd
	rm -fr $(SDHCI_ROOT)/target/sim/vsim/tb_dat.vcd
	rm -fr $(SDHCI_ROOT)/target/sim/vsim/transcript

sdhci-sim-model-clean:
	rm -f $(SDHCI_ROOT)/target/sim/model/sd_crc_7.v
	rm -f $(SDHCI_ROOT)/target/sim/model/sd_crc_16.v
	rm -f $(SDHCI_ROOT)/target/sim/model/sdModel.v

sdhci-sim-clean: sdhci-sim-vsim-clean
sdhci-sim-deepclean: sdhci-sim-clean sdhci-sim-model-clean

.PHONY: sdhci-sim-all sdhci-sim-clean sdhci-sim-deepclean sdhci-sim-model-clean sdhci-sim-vsim-clean
