
// TODOs
// implement SRAM backpressure into FIFO

// This module acts as the top module for the acquisition
// pipeline. it allows for transfering the aquisition data
// into SRAM and then further, note it's a manager
// note it directly handles the CDC

module adc_acquisition_top import adc_acquisition_pkg::*; #(
  /// The OBI configuration connected to this peripheral.
  parameter obi_pkg::obi_cfg_t ObiSbrCfg = obi_pkg::ObiDefaultConfig, // SbrObiCfg
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
  // registers (w obi connection) //
  //////////////////////////////////////
  logic [adc_acquisition_reg_pkg::ADC_ACQUISITION_REG_MIN_ADDR_WIDTH-1:0] sbr_obi_req_relative_addr;
  reg2hw_t hw2reg;
  hw2reg_t reg2hw;
  adc_acquisition_reg #(
    .ID_WIDTH(ObiSbrCfg.IdWidth)
  ) i_adc_acquisition_reg (
    .clk( clk_i ),
    .rst( ~rst_ni ),
    // Hardware register interface
    .hwif_in  ( hw2reg ),
    .hwif_out ( reg2hw ),
    // OBI
    .s_obi_req    ( sbr_obi_req_i.req ),
    .s_obi_gnt    ( sbr_obi_rsp_o.gnt ),
    .s_obi_addr   ( sbr_obi_req_relative_addr ),
    .s_obi_we     ( sbr_obi_req_i.a.we ),
    .s_obi_be     ( sbr_obi_req_i.a.be ),
    .s_obi_wdata  ( sbr_obi_req_i.a.wdata ),
    .s_obi_aid    ( sbr_obi_req_i.a.aid ),
    .s_obi_rvalid ( sbr_obi_rsp_o.rvalid ),
    .s_obi_rready ( 1 ),
    .s_obi_rdata  ( sbr_obi_rsp_o.r.rdata ),
    .s_obi_err    ( sbr_obi_rsp_o.r.err ),
    .s_obi_rid    ( sbr_obi_rsp_o.r.rid )
  );

  assign sbr_obi_req_relative_addr  = (sbr_obi_req_i.a.addr - ADC_REGISTER_BASE_ADDRESS);
  assign sbr_obi_rsp_o.r.r_optional = '0;

  //////////////////////////////////////
  // CDC for adc inputs //
  //////////////////////////////////////
  logic adc_data_valid_sync;
  adc_data_raw_t adc_data_sync;
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
  logic adc_data_soft_rst;
  logic adc_data_word_ready;
  adc_data_word_t adc_data_word;
  logic [ADC_BIT_WIDTH-1:0] adc_data_unpacked [1:0];
  adc_acquisition_fifo #(
    .DATA_WIDTH ( ADC_BIT_WIDTH ),
    .BATCH_SIZE ( 2 ),
    .FIFO_BATCH_DEPTH ( 4 )
  ) i_adc_acquisition_fifo (
    .clk_i, .rst_ni,
    .soft_rst_i ( 0 ),
    .read_i     ( adc_data_word_ready ),
    .write_i    ( adc_data_valid_sync ),
    .data_i     ( adc_data_sync ),
    .valid_o    ( adc_data_word_ready ),
    .overflow_o (),
    .data_o     ( adc_data_unpacked )
  );
  // pack the data into a word
  assign adc_data_word = {2'b00, adc_data_unpacked[1], 2'b00, adc_data_unpacked[0]};

  //////////////////////////////////////
  // control logic //
  //////////////////////////////////////
  logic        dma_push;     // signal to push the data
  logic [31:0] dma_data;     // data
  logic [31:0] dma_address;  // address
  always_comb begin : adc_control_logic
    // ADC Data Fifo Control
    adc_data_soft_rst = 0;
    // Default DMA control
    dma_push    = '0;
    dma_data    = 'x;
    dma_address = 'x;
    // Latch the data
    hw2reg.STATUS.MODE.next             = reg2hw.STATUS.MODE.value;
    hw2reg.STATUS.F0_FULL.next          = reg2hw.STATUS.F0_FULL.value;
    hw2reg.STATUS.F1_FULL.next          = reg2hw.STATUS.F1_FULL.value;
    hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value;
    hw2reg.CNTRL.RESET_WRITE_HEAD.next  = reg2hw.CNTRL.RESET_WRITE_HEAD.value;

    if(reg2hw.CNTRL.RESET_WRITE_HEAD.value) begin
      hw2reg.CNTRL.RESET_WRITE_HEAD.next  = '0;
      hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.F0_START_ADDR.WORD_ADDRESS.value;
    end

    case (reg2hw.STATUS.MODE.value)
      adc_acquisition_reg_pkg::adc_mode__IDLE: begin
        adc_data_soft_rst = 1;
      end
      adc_acquisition_reg_pkg::adc_mode__SINGLE_ACQ_F0: begin
        // Set the data as output
        dma_push    = adc_data_word_ready;
        dma_address = {reg2hw.WRITE_HEAD.WORD_ADDRESS.value, 2'b00};
        dma_data    = adc_data_word;
        // Increment counter if we write
        if(dma_push)
          hw2reg.WRITE_HEAD.WORD_ADDRESS.next = reg2hw.WRITE_HEAD.WORD_ADDRESS.value + 1;
        // Finished Frame
        if(reg2hw.WRITE_HEAD.WORD_ADDRESS.value==reg2hw.F0_END_ADDR.WORD_ADDRESS.value) begin
          hw2reg.STATUS.F0_FULL.next = 1'b1;
          hw2reg.STATUS.MODE.next    = adc_acquisition_reg_pkg::adc_mode__IDLE;
        end
      end
      adc_acquisition_reg_pkg::adc_mode__CONTINUOUS_ACQ_F0_F1: begin
      end
      default: ;
    endcase

  end

  //////////////////////////////////////
  // dma obi translator //
  //////////////////////////////////////
  always_comb begin : adc_dma_obi_translator
    mgr_obi_req_o.req     = dma_push;
    mgr_obi_req_o.a.addr  = dma_address;
    mgr_obi_req_o.a.we    = dma_push;
    mgr_obi_req_o.a.be    = 4'hf;
    mgr_obi_req_o.a.wdata = dma_data;
    mgr_obi_req_o.a.aid   = '0;
  end

endmodule
