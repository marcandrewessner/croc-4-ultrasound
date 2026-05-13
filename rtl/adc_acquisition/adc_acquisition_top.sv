

// This module acts as the top module for the acquisition
// pipeline. it allows for transfering the aquisition data
// into SRAM and then further, note it's a manager
// note it directly handles the CDC

module adc_acquisition_top import adc_acquisition_pkg::*; #(
  /// OBI MGR ports
  parameter type mgr_obi_req_t = logic,
  parameter type mgr_obi_rsp_t = logic,
  /// OBI SBR ports
  parameter type sbr_obi_req_t = logic,
  parameter type sbr_obi_rsp_t = logic
) (
  input logic clk_i,
  input logic rst_ni,

  // OBI
  output mgr_obi_req_t mgr_obi_req_o,
  input  mgr_obi_rsp_t mgr_obi_rsp_i,
  input  sbr_obi_req_t sbr_obi_req_i,
  output sbr_obi_rsp_t sbr_obi_rsp_o,

  // Interrupts
  output logic interrupt_frame_full_o,

  // Outer world pins unsynchronized
  input adc_input_signals_t adc_input_signals
);

  //////////////////////////////////////
  // prepare signals //
  //////////////////////////////////////
  // CDC Outputs
  logic adc_data_valid_sync;
  adc_data_raw_t adc_data_sync;
  // Packing data into words
  logic has_adc_data_word;
  adc_data_word_t adc_data_word;

  //////////////////////////////////////
  // CDC for adc inputs //
  //////////////////////////////////////
  cdc_fifo_gray #(
    .WIDTH( ADC_BIT_WIDTH )
  ) i_cdc_fifo_gray (
    .src_clk_i   ( adc_input_signals.clk ),
    .src_rst_ni  ( adc_input_signals.rst_n ),
    .src_valid_i ( adc_input_signals.valid ),
    .src_data_i  ( adc_input_signals.data ),
    .src_ready_o (),

    .dst_clk_i   ( clk_i ),
    .dst_rst_ni  ( rst_ni ),
    .dst_ready_i ( 1 ),
    .dst_data_o  ( adc_data_sync ),
    .dst_valid_o ( adc_data_valid_sync )
  );

  //////////////////////////////////////
  // read the data into batched fifo //
  //////////////////////////////////////
  logic [ADC_BIT_WIDTH-1:0] adc_data_tmp [1:0];
  adc_acquisition_fifo #(
    .DATA_WIDTH ( ADC_BIT_WIDTH ),
    .BATCH_SIZE ( 2 ),
    .FIFO_BATCH_DEPTH ( 4 )
  ) i_adc_acquisition_fifo (
    .clk_i, .rst_ni,
    .soft_rst_i ( 0 ),
    .read_i     ( has_adc_data_word ),
    .write_i    ( adc_data_valid_sync ),
    .data_i     ( adc_data_sync ),
    .valid_o    ( has_adc_data_word ),
    .overflow_o (),
    .data_o     ( adc_data_tmp )
  );
  assign adc_data_word = {2'b00, adc_data_tmp[1], 2'b00, adc_data_tmp[0]};

  //////////////////////////////////////
  // push data directly into sram //
  //////////////////////////////////////
  localparam int SRAM_BASE_ADDRESS = 32'h1000_1000;
  int address_d, address_q;

  `FF(address_q, address_d, SRAM_BASE_ADDRESS, clk_i, rst_ni)

  always_comb begin : adc_dma_logic
    interrupt_frame_full_o = address_q >= SRAM_BASE_ADDRESS+'d20;
    address_d = address_q + (has_adc_data_word && !interrupt_frame_full_o ? 1 : 0);
    
    mgr_obi_req_o.req     = has_adc_data_word;
    mgr_obi_req_o.a.addr  = address_q;
    mgr_obi_req_o.a.we    = has_adc_data_word;
    mgr_obi_req_o.a.be    = 4'hf;
    mgr_obi_req_o.a.wdata = adc_data_word;
    mgr_obi_req_o.a.aid   = 'he;

    // Just throw an interrupt after a recording of 200 samples = 100 words
  end


endmodule
