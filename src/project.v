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
  
  // UART Configuration
  // For simulation with 100kHz clock (10us period) and 10k Baud:
  // CLKS_PER_BIT_UART = 100_000 Hz / 10_000 Baud = 10
  // For Arty A7 with 100MHz clock and 115200 Baud:
  // CLKS_PER_BIT_UART = 100_000_000 / 115200 = 868 (approx)
  // Choose parameters based on your target system. For this testbench, we'll use sim values.
  localparam SIM_CLOCK_FREQ_HZ = 100_000;
  localparam SIM_BAUD_RATE     = 10_000;
  localparam CLKS_PER_BIT_FOR_UART = (SIM_CLOCK_FREQ_HZ / SIM_BAUD_RATE);

  wire uart_rx_serial_line = ui_in[0]; // Use ui_in[0] as the UART RX pin
  wire [7:0] uart_received_byte;
  wire       uart_byte_is_ready;


  uart_receiver_custom #(
      .CLKS_PER_BIT(CLKS_PER_BIT_FOR_UART)
  ) uart_rx_inst (
      .clk(clk),
      .rst_n(rst_n),
      .rx_serial_in(uart_rx_serial_line),
      .data_out(uart_received_byte),
      .byte_ready(uart_byte_is_ready)
  );

  // Internal state definitions
  localparam STATE_WAIT_UART_INPUT = 2'd0,
             STATE_COMPUTE         = 2'd1,
             STATE_OUTPUT          = 2'd2;

  reg [1:0] state;
  reg [1:0] counter;
  reg [2:0] input_byte_counter; // Counts 0-7 for 8 bytes (A0-A3, then B0-B3)
  reg [1:0] output_cell_counter; // used for output counting (0-3)
  reg [7:0] A0, A1, A2, A3;   // Elements for matrix A (rowwise order)
  reg [7:0] B0, B1, B2, B3;   // Elements for matrix B (rowwise order)
  reg [15:0] C00, C01, C10, C11; // Full-precision intermediate products
  reg [7:0] uo_out_reg;       // Registered output (truncated to 8 bits)

  // Output assignment
  assign uo_out = uo_out_reg;

  // Sequential logic with state machine
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state               <= STATE_WAIT_UART_INPUT;
      input_byte_counter  <= 0;
      output_cell_counter <= 0;
      A0 <= 0; A1 <= 0; A2 <= 0; A3 <= 0;
      B0 <= 0; B1 <= 0; B2 <= 0; B3 <= 0;
      C00 <= 0; C01 <= 0; C10 <= 0; C11 <= 0;
      uo_out_reg <= 0;
    end else begin
      case (state)
        STATE_WAIT_UART_INPUT: begin
          if (uart_byte_is_ready) begin
            case (input_byte_counter)
              3'd0: A0 <= uart_received_byte;
              3'd1: A1 <= uart_received_byte;
              3'd2: A2 <= uart_received_byte;
              3'd3: A3 <= uart_received_byte;
              3'd4: B0 <= uart_received_byte;
              3'd5: B1 <= uart_received_byte;
              3'd6: B2 <= uart_received_byte;
              3'd7: B3 <= uart_received_byte;
            endcase
            if (input_byte_counter == 3'd7) begin
              state              <= STATE_COMPUTE;
              input_byte_counter <= 0; // Reset for next transaction
            end else begin
              input_byte_counter <= input_byte_counter + 1;
            end
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
            state   <= STATE_WAIT_UART_INPUT;
            counter <= 0;
          end else begin
            counter <= counter + 1;
          end
        end

          default: state <= STATE_WAIT_UART_INPUT;
      endcase
    end
  end

  // List all unused input signals to prevent warnings
  wire _unused = &{ena, ui_in[7:1], uio_in};

endmodule