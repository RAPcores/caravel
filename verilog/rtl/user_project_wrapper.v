`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * An example user project is provided in this wrapper.  The
 * example should be removed and replaced with the actual
 * user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
)(
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oen,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7.
    inout [`MPRJ_IO_PADS-8:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2
);

    /*--------------------------------------*/
    /* User project is instantiated  here   */
    /*--------------------------------------*/

    top mprj (
	.vdda1(vdda1),	// User area 1 3.3V power
	.vdda2(vdda2),	// User area 2 3.3V power
	.vssa1(vssa1),	// User area 1 analog ground
	.vssa2(vssa2),	// User area 2 analog ground
	.vccd1(vccd1),	// User area 1 1.8V power
	.vccd2(vccd2),	// User area 2 1.8V power
	.vssd1(vssd1),	// User area 1 digital ground
	.vssd2(vssd2),	// User area 2 digital ground

	// MGMT core clock and reset

    	.wb_clk_i(wb_clk_i),
    	.wb_rst_i(wb_rst_i),

	// MGMT SoC Wishbone Slave

	.wbs_cyc_i(wbs_cyc_i),
	.wbs_stb_i(wbs_stb_i),
	.wbs_we_i(wbs_we_i),
	.wbs_sel_i(wbs_sel_i),
	.wbs_adr_i(wbs_adr_i),
	.wbs_dat_i(wbs_dat_i),
	.wbs_ack_o(wbs_ack_o),
	.wbs_dat_o(wbs_dat_o),

	// Logic Analyzer

	.la_data_in(la_data_in),
	.la_data_out(la_data_out),
	.la_oen (la_oen),

	// IO Pads

	.io_in (io_in),
    	.io_out(io_out),
    	.io_oeb(io_oeb)
    );

endmodule	// user_project_wrapper
`default_nettype wire

`default_nettype none

`include "macro_params.v"
`include "constants.v"
`include "stepper.v"
`include "spi.v"
`include "quad_enc.v"
`include "pwm.v"
`include "microstepper/microstepper_top.v"
//`include "microstepper2/stepper_top.v"

module top (
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oen,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7.
    inout [`MPRJ_IO_PADS-8:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2
);

    wire CLK = wb_clk_i; // input
    wire SCK = io_in[1]; // input
    wire CS = io_in[2];
    wire COPI = io_in[3];
    wire CIPO = io_out[4];

    //`ifdef ULTIBRIDGE
    wire CHARGEPUMP = io_out[5];
    wire analog_cmp1 = io_in[6];
    wire analog_out1 = io_out[7];
    wire analog_cmp2 = io_in[8];
    wire analog_out2 = io_out[9];
    wire PHASE_A1 = io_out[10];
    wire PHASE_A2 = io_out[11];
    wire PHASE_B1 = io_out[12];
    wire PHASE_B2 = io_out[13];
    wire PHASE_A1_H = io_out[14];
    wire PHASE_A2_H = io_out[15];
    wire PHASE_B1_H = io_out[16];
    wire PHASE_B2_H = io_out[17];
    `ifdef QUAD_ENC
        input [`QUAD_ENC:1] ENC_B,
        input [`QUAD_ENC:1] ENC_A,
    `endif
    //`ifdef BUFFER_DTR
    //    output BUFFER_DTR,
    //`endif
    //`ifdef MOVE_DONE
    //    output MOVE_DONE,
    //`endif
    //`ifdef HALT
    //    input HALT,
    //`endif

  // Word handler
  // The system operates on 64 bit little endian words
  // This should make it easier to send 64 bit chunks from the host controller
  reg [63:0] word_send_data;
  reg [63:0] word_data_received;
  wire word_received;
  SPIWord word_proc (
                .clk(CLK), //.clk(spi_clock),
                .SCK(SCK),
                .CS(CS),
                .COPI(COPI),
                .CIPO(CIPO),
                .word_send_data(word_send_data),
                .word_received(word_received),
                .word_data_received(word_data_received));

  //Reset
  wire resetn;
  reg [7:0] resetn_counter = 0;
  assign resetn = &resetn_counter;
  always @(posedge CLK) begin
    if (!resetn) resetn_counter <= resetn_counter + 1'b1;
  end
  wire reset = resetn;

  // Stepper Setup
  // TODO: Generate statement?
  reg [2:0] microsteps = 2;
  reg [7:0] current = 140;
  wire step;
  wire dir;
  reg enable;

    microstepper_top microstepper0(
      .clk( spi_clock),
      .resetn( resetn),
      .s_l ({PHASE_B2, PHASE_B1, PHASE_A2, PHASE_A1}),
      .s_h ({PHASE_B2_H, PHASE_B1_H, PHASE_A2_H, PHASE_A1_H}),
      .analog_cmp1 (analog_cmp1),
      .analog_out1 (analog_out1),
      .analog_cmp2 (analog_cmp2),
      .analog_out2 (analog_out2),
      .chargepump_pin (CHARGEPUMP),
      .step (step),
      .dir (dir),
      .enable(enable),
      );
  `endif


  //
  // Encoder
  //
  reg signed [63:0] encoder_count;
  reg signed [63:0] encoder_store; // Snapshot for SPI comms
  reg [7:0] encoder_multiplier = 1;
  wire encoder_fault;
  `ifdef QUAD_ENC
    // TODO: For ... generate
    quad_enc #(.encbits(64)) encoder0
    (
      .resetn(reset),
      .clk(CLK),
      .a(ENC_A[1]),
      .b(ENC_B[1]),
      .faultn(encoder_fault),
      .count(encoder_count),
      .multiplier(encoder_multiplier));
  `endif

  //
  // State Machine for handling SPI Messages
  //

  reg [7:0] message_word_count = 0;
  reg [7:0] message_header;
  reg [`MOVE_BUFFER_BITS:0] writemoveind = 0;

  // check if the Header indicated multi-word transfer
  wire awaiting_more_words = (message_header == `CMD_COORDINATED_STEP) |
                             (message_header == `CMD_API_VERSION);

  always @(posedge word_received) begin

    // Zero out send data register
    word_send_data <= 64'b0;

    // Header Processing
    if (!awaiting_more_words) begin

      // Save CMD header incase multi word transaction
      message_header <= word_data_received[63:56]; // Header is 8 MSB

      // First word so message count zero
      message_word_count <= 1;

      case (word_data_received[63:56])

        // Coordinated Move
        `CMD_COORDINATED_STEP: begin

          // Get Direction Bits
          dir_r[writemoveind] <= word_data_received[0];

          // Store encoder values across all axes Now
          encoder_store <= encoder_count;

        end

        // Motor Enable/disable
        `CMD_MOTOR_ENABLE: begin
          enable <= word_data_received[0];
        end

        // Clock divisor (24 bit)
        `CMD_CLK_DIVISOR: begin
          clock_divisor[7:0] <= word_data_received[7:0];
        end

        // Set Microstepping
        `CMD_MOTORCONFIG: begin
          // TODO needs to be power of two
          current[7:0] <= word_data_received[15:8];
          microsteps[2:0] <= word_data_received[2:0];
        end

        // API Version
        `CMD_API_VERSION: begin
          word_send_data[7:0] <= `VERSION_PATCH;
          word_send_data[15:8] <= `VERSION_MINOR;
          word_send_data[23:16] <= `VERSION_MAJOR;
        end
      endcase

    // Addition Word Processing
    end else begin

      message_word_count <= message_word_count + 1;

      case (message_header)
        // Move Routine
        `CMD_COORDINATED_STEP: begin
          // the first non-header word is the move duration
          case (message_word_count)
            1: begin
              move_duration[writemoveind][63:0] <= word_data_received[63:0];
              //word_send_data[63:0] = last_steps_taken[63:0]; // Prep to send steps
            end
            2: begin
              increment[writemoveind][63:0] <= word_data_received[63:0];
              word_send_data[63:0] <= encoder_store[63:0]; // Prep to send encoder read
            end
            3: begin
                incrementincrement[writemoveind][63:0] <= word_data_received[63:0];
                message_word_count <= 0;
                stepready[writemoveind] <= ~stepready[writemoveind];
                writemoveind <= writemoveind + 1'b1;
                message_header <= 8'b0; // Reset Message Header
                `ifdef FORMAL
                  assert(writemoveind <= `MOVE_BUFFER_SIZE);
                `endif
            end
          endcase
        end
      endcase
    end
  end

  //
  // Stepper Timing Routine
  //

  // coordinated move execution

  reg [`MOVE_BUFFER_BITS:0] moveind = 0; // Move index cursor

  // Latching mechanism for engaging the buffered move.
  reg [`MOVE_BUFFER_SIZE:0] stepready;
  reg [`MOVE_BUFFER_SIZE:0] stepfinished;

  reg [63:0] move_duration [`MOVE_BUFFER_SIZE:0];
  reg [7:0] clock_divisor = 40;  // should be 40 for 400 khz at 16Mhz Clk
  reg [`MOVE_BUFFER_SIZE:0] dir_r;

  reg [63:0] tickdowncount;  // move down count (clock cycles)
  reg [7:0] clkaccum = 8'b1;  // intra-tick accumulator

  reg signed [63:0] substep_accumulator = 0; // typemax(Int64) - 100 for buffer
  reg signed [63:0] increment_r;
  reg signed [63:0] increment [`MOVE_BUFFER_SIZE:0];
  reg signed [63:0] incrementincrement [`MOVE_BUFFER_SIZE:0];

  reg finishedmove = 1; // flag inidicating a move has been finished, so load next
  wire processing_move = (stepfinished[moveind] ^ stepready[moveind]);
  wire loading_move = finishedmove & processing_move;
  wire executing_move = !finishedmove & processing_move;

  // Implement flow control and event pins if specified
  `ifdef BUFFER_DTR
    assign BUFFER_DTR = ~(~stepfinished == stepready);
  `endif

  `ifdef MOVE_DONE
    reg move_done_r = 0;
    assign MOVE_DONE = move_done_r;
    always @(posedge finishedmove)
      move_done_r = ~move_done_r;
  `endif

  assign dir = dir_r[moveind]; // set direction
  assign step = (substep_accumulator > 0);

  always @(posedge CLK) begin

    // HALT line (active low) then reset buffer latch and index
    // TODO: Should substep accumulator reset?
    `ifdef HALT
      if (!HALT) begin
        moveind <= writemoveind; // match buffer cursor
        stepfinished <= stepready; // reset latch
        finishedmove <= 1; // Puts us back in loading_move
      end
    `endif

    // Load up the move duration
    if (loading_move) begin
      tickdowncount <= move_duration[moveind];
      finishedmove <= 0;
      increment_r <= increment[moveind];
    end

    // check if this move has been done before
    if(executing_move) begin

      // Step taken, rollback accumulator
      if (substep_accumulator > 0) begin
        substep_accumulator <= substep_accumulator - 64'h7fffffffffffff9b;
      end

      // DDA clock divisor
      clkaccum <= clkaccum - 8'b1;
      if (clkaccum == 8'b0) begin

        increment_r <= increment_r + incrementincrement[moveind];
        substep_accumulator <= substep_accumulator + increment_r;

        // Increment tick accumulators
        clkaccum <= clock_divisor;
        tickdowncount <= tickdowncount - 1'b1;
        // See if we finished the segment and incrment the buffer
        if(tickdowncount == 0) begin
          stepfinished[moveind] <= ~stepfinished[moveind];
          moveind <= moveind + 1'b1;
          finishedmove <= 1;
          `ifdef FORMAL
            assert(moveind <= `MOVE_BUFFER_SIZE);
          `endif
        end
      end
    end
  end
endmodule
