

// This module acts as the top module for the acquisition
// pipeline. it allows for transfering the aquisition data
// into SRAM and then further, note it's a manager
// note it directly handles the CDC

module adc_acquisition_top import adc_acquisition_pkg::*; #(
  /// OBI MGR ports
  parameter type obi_mgr_req_t = logic;
  parameter type obi_mgr_rsp_t = logic;
  /// OBI SBR ports
  parameter type obi_sbr_req_t = logic;
  parameter type obi_sbr_rsp_t = logic;
) (

);

endmodule
