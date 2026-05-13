
// This is the package for the acquisition chain and data

package adc_acquisition_pkg;

  //////////////////////////////////////
  // register types //
  //////////////////////////////////////
  import adc_acquisition_reg_pkg::*;
  typedef adc_acquisition_reg__in_t  reg2hw_t;
  typedef adc_acquisition_reg__out_t hw2reg_t;

  //////////////////////////////////////
  // system configuration //
  //////////////////////////////////////
  localparam int ADC_BIT_WIDTH = 14;
  localparam int ADC_REGISTER_BASE_ADDRESS = 32'h0300_C000;

  //////////////////////////////////////
  // type definitions //
  //////////////////////////////////////
  // Raw input signals
  typedef logic [ADC_BIT_WIDTH-1:0] adc_data_raw_t;

  typedef struct packed {
    logic          clk;
    logic          rst_n;
    logic          valid;
    adc_data_raw_t data;
  } adc_input_signals_t;

  // Packed and aligned data => 16bit
  typedef struct packed {
    logic [16-ADC_BIT_WIDTH-1:0] unused;
    adc_data_raw_t               data;
  } adc_data_t;

  // Create an ADC data word => 32bit
  typedef struct packed {
    adc_data_t upper; //
    adc_data_t lower; // 
  } adc_data_word_t;

endpackage