/*
 * Copyright (c) 2024 Agneya Tharun 
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_2x2MatrixMult_Vort3xed (
    input  wire [7:0] ui_in,    // Input for first matrix values
    output wire [7:0] uo_out,   // Sequential output of result cells
    input  wire [7:0] uio_in,   // Input for second matrix values
    output wire [7:0] uio_out,  // (Unused)
    output wire [7:0] uio_oe,   // (Unused)
    input  wire       ena,      // always 1 when design powered
    input  wire       clk,      // clock
    input  wire       rst_n     // active-low reset
);

  // Unused outputs fixed to 0.
  assign uio_out = 0;
  assign uio_oe  = 0;

  // Internal state definitions
  localparam STATE_INPUT_A = 2'd0,
             STATE_INPUT_B = 2'd1,
             STATE_COMPUTE = 2'd2,
             STATE_OUTPUT  = 2'd3;

  reg [1:0] state;
  reg [1:0] counter;      // used for both input and output counting
  reg [7:0] A0, A1, A2, A3;   // Elements for matrix A (rowwise order)
  reg [7:0] B0, B1, B2, B3;   // Elements for matrix B (rowwise order)
  reg [15:0] C00, C01, C10, C11; // Full-precision intermediate products
  reg [7:0] uo_out_reg;       // Registered output (truncated to 8 bits)

  // Output assignment
  assign uo_out = uo_out_reg;

  // Sequential logic with state machine
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= STATE_INPUT_A;
      counter    <= 0;
      A0 <= 0; A1 <= 0; A2 <= 0; A3 <= 0;
      B0 <= 0; B1 <= 0; B2 <= 0; B3 <= 0;
      C00 <= 0; C01 <= 0; C10 <= 0; C11 <= 0;
      uo_out_reg <= 0;
    end else begin
      case (state)
        // Input first matrix (from ui_in)
        STATE_INPUT_A: begin
          case (counter)
            2'd0: A0 <= ui_in;
            2'd1: A1 <= ui_in;
            2'd2: A2 <= ui_in;
            2'd3: A3 <= ui_in;
          endcase
          if (counter == 2'd3) begin
            state   <= STATE_INPUT_B;
            counter <= 0;
          end else begin
            counter <= counter + 1;
          end
        end

        // Input second matrix (from uio_in)
        STATE_INPUT_B: begin
          case (counter)
            2'd0: B0 <= uio_in;
            2'd1: B1 <= uio_in;
            2'd2: B2 <= uio_in;
            2'd3: B3 <= uio_in;
          endcase
          if (counter == 2'd3) begin
            state   <= STATE_COMPUTE;
            counter <= 0;
          end else begin
            counter <= counter + 1;
          end
        end

        // Compute matrix multiplication
        STATE_COMPUTE: begin
          // Standard 2x2 matrix multiply (results are truncated to 8 bits)
          C00 <= A0 * B0 + A1 * B2;
          C01 <= A0 * B1 + A1 * B3;
          C10 <= A2 * B0 + A3 * B2;
          C11 <= A2 * B1 + A3 * B3;
          state <= STATE_OUTPUT;
          counter <= 0;
        end

        // Output each cell over successive clock cycles
        STATE_OUTPUT: begin
          case (counter)
            2'd0: uo_out_reg <= C00[7:0];
            2'd1: uo_out_reg <= C01[7:0];
            2'd2: uo_out_reg <= C10[7:0];
            2'd3: uo_out_reg <= C11[7:0];
          endcase
          if (counter == 2'd3) begin
            // After outputting all cells, restart input (or hold, depending on design needs)
            state   <= STATE_INPUT_A;
            counter <= 0;
          end else begin
            counter <= counter + 1;
          end
        end

        default: state <= STATE_INPUT_A;
      endcase
    end
  end

  // List all unused input signals to prevent warnings
  wire _unused = &{ena};

endmodule