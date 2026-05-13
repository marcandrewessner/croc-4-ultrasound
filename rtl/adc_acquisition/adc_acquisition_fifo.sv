

`include "common_cells/registers.svh"


module adc_acquisition_fifo import adc_acquisition_pkg::*; #(
  // Define the width of one data unit
  parameter int DATA_WIDTH = 1,
  // Define how big the batch size is
  parameter int BATCH_SIZE = 2,
  // Define how deep (in batches) the FIFO is
  parameter int FIFO_BATCH_DEPTH = 2
)(
  input logic clk_i,
  input logic rst_ni,

  input logic soft_rst_i,               // reset fifo: write & read head at 0 and no data available
  input logic read_i,                   // read the data move the read head
  input logic write_i,                  // write enabled
  input logic [DATA_WIDTH-1:0] data_i,  // the input data

  output logic valid_o,    // data is ready to be read
  output logic overflow_o, // fifo is full and has overflown
  output logic [DATA_WIDTH-1:0] data_o [BATCH_SIZE-1:0]
);

  //////////////////////////////////////
  // signal preparation //
  //////////////////////////////////////
  logic overflow, valid;
  // states
  int rec_data_count_d, rec_data_count_q;
  int      read_head_d, read_head_q;
  int     write_head_d, write_head_q;

  // Fifo data container
  logic [DATA_WIDTH-1:0] fifo_data_d [BATCH_SIZE*FIFO_BATCH_DEPTH-1:0];
  logic [DATA_WIDTH-1:0] fifo_data_q [BATCH_SIZE*FIFO_BATCH_DEPTH-1:0];

  // Flipflops q <= d;
  `FF(rec_data_count_q, rec_data_count_d, '0, clk_i, rst_ni)
  `FF(read_head_q, read_head_d, '0, clk_i, rst_ni)
  `FF(write_head_q, write_head_d, '0, clk_i, rst_ni)
  `FF(fifo_data_q, fifo_data_d, '{default:'0}, clk_i, rst_ni)

  //////////////////////////////////////
  // state transfer logic  //
  //////////////////////////////////////
  always_comb begin : state_transfer_logic
    rec_data_count_d = rec_data_count_q;
    read_head_d      = read_head_q;
    write_head_d     = write_head_q;
    fifo_data_d      = fifo_data_q;

    // derivations
    overflow = rec_data_count_q > BATCH_SIZE*FIFO_BATCH_DEPTH;
    valid    = rec_data_count_q >= BATCH_SIZE;

    if(soft_rst_i) begin
      // Check for the soft reset
      rec_data_count_d = '0;
      read_head_d      = '0;
      write_head_d     = '0;
    end else begin
      // Create the variable for the rec counter
      int next_rec_data_counter;
      next_rec_data_counter = rec_data_count_q;
      // Normal operation
      if(write_i && !overflow) begin
        write_head_d = (write_head_q==BATCH_SIZE*FIFO_BATCH_DEPTH-1) ? '0 : write_head_q + 1;
        fifo_data_d[write_head_q] = data_i;
        next_rec_data_counter = next_rec_data_counter + 1;
      end
      if(read_i && valid) begin
        read_head_d = (read_head_q==BATCH_SIZE*FIFO_BATCH_DEPTH-1*BATCH_SIZE) ? '0 : read_head_q + BATCH_SIZE;
        next_rec_data_counter = next_rec_data_counter - BATCH_SIZE;
      end
      // Push the rec_data_counter
      rec_data_count_d = next_rec_data_counter;
    end
  end


  //////////////////////////////////////
  // output logic  //
  //////////////////////////////////////
  always_comb begin : output_logic
    overflow_o = overflow;
    valid_o    = valid;
    // set the batches
    for(int i=0; i<BATCH_SIZE; i++) begin
      int read_offset;
      read_offset = (read_head_q+i >= BATCH_SIZE*FIFO_BATCH_DEPTH) ? read_head_q+i-BATCH_SIZE*FIFO_BATCH_DEPTH : read_head_q+i;
      data_o[i] = fifo_data_q[read_offset];
    end
  end

endmodule