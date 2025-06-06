/*
 * Copyright (c) 2024 Agneya Tharun 
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_2x2MatrixMult_Vort3xed (
    input  wire [7:0] ui_in,    // Input for first matrix values (ui_in[0] for UART RX)
    output wire [7:0] uo_out,   // Sequential output of result cells (parallel)
    input  wire [7:0] uio_in,   // (Unused by this design's logic)
    output wire [7:0] uio_out,  // uio_out[0] for UART TX
    output wire [7:0] uio_oe,   // uio_oe[0] to enable UART TX
    input  wire       ena,      // always 1 when design powered
    input  wire       clk,      // clock
    input  wire       rst_n     // active-low reset
);

  // uio_out[7:1] and uio_oe[7:1] are unused and fixed to 0.
  assign uio_out[7:1] = 7'b0;
  assign uio_oe[7:1]  = 7'b0;

  // UART Configuration
  // localparam SIM_CLOCK_FREQ_HZ = 200_000; // For simulation
  // localparam SIM_BAUD_RATE     = 10_000;  // For simulation
  // localparam CLKS_PER_BIT_FOR_UART = (SIM_CLOCK_FREQ_HZ / SIM_BAUD_RATE);

  // For Arty A7 with 100MHz clock and 120000 Baud:
  localparam REAL_CLOCK_FREQ_HZ = 100_000_000;
  localparam REAL_BAUD_RATE = 120000;
  localparam CLKS_PER_BIT_FOR_UART = (REAL_CLOCK_FREQ_HZ / REAL_BAUD_RATE);

  //print out the CLKS_PER_BIT_FOR_UART value
  initial begin
    $display("CLKS_PER_BIT_FOR_UART = %d", CLKS_PER_BIT_FOR_UART
    );
  end

  // UART Receiver instance for input
  wire       uart_rx_serial_line = ui_in[0];  // Use ui_in[0] as the UART RX pin
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

  // UART Transmitter instance for output
  wire uart_tx_serial_pin;
  wire uart_tx_busy;
  reg uart_tx_start_pulse;
  reg [7:0] uart_tx_data_reg;

  uart_transmitter_custom #(
      .CLKS_PER_BIT(CLKS_PER_BIT_FOR_UART)
  ) uart_tx_inst (
      .clk(clk),
      .rst_n(rst_n),
      .tx_start(uart_tx_start_pulse),
      .data_in(uart_tx_data_reg),
      .tx_serial_out(uart_tx_serial_pin),
      .tx_busy(uart_tx_busy)
  );

  assign uio_out[0] = uart_tx_serial_pin;
  reg uio_oe_bit0_reg;  // Controls uio_oe[0] for UART TX
  assign uio_oe[0] = uio_oe_bit0_reg;

  // Internal state definitions
  localparam STATE_WAIT_UART_INPUT = 4'd0,
  STATE_COMPUTE = 4'd1, STATE_LOAD_UART_DATA = 4'd2,
  STATE_TRIGGER_UART_SEND = 4'd3,
  STATE_WAIT_TX_COMPLETE = 4'd4;  // Waits for current UART TX to finish

  reg [3:0] state;
  reg [2:0] input_byte_counter;  // Counts 0-7 for 8 input bytes
  reg [1:0] output_byte_counter;  // Counts 0-3 for 4 output bytes (C00, C01, C10, C11)

  reg [7:0] A0, A1, A2, A3;  // Elements for matrix A
  reg [7:0] B0, B1, B2, B3;  // Elements for matrix B
  reg [15:0] C00, C01, C10, C11;  // Full-precision intermediate products
  reg [7:0] uo_out_reg;  // Registered output for uo_out (parallel observation)

  // Output assignment for parallel port
  assign uo_out = uo_out_reg;

  // Sequential logic with state machine
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state               <= STATE_WAIT_UART_INPUT;
      input_byte_counter  <= 0;
      output_byte_counter <= 0;
      A0                  <= 0;
      A1                  <= 0;
      A2                  <= 0;
      A3                  <= 0;
      B0                  <= 0;
      B1                  <= 0;
      B2                  <= 0;
      B3                  <= 0;
      C00                 <= 0;
      C01                 <= 0;
      C10                 <= 0;
      C11                 <= 0;
      uo_out_reg          <= 0;
      uart_tx_start_pulse <= 1'b0;
      uart_tx_data_reg    <= 0;
      uio_oe_bit0_reg     <= 1'b0;  // uio_out[0] is input by default
    end else begin
      // Default actions for pulses/control signals
      uart_tx_start_pulse <= 1'b0;  // tx_start is a pulse

      case (state)
        STATE_WAIT_UART_INPUT: begin
          uio_oe_bit0_reg <= 1'b0;  // Ensure UART TX pin is input
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
              input_byte_counter <= 0;  // Reset for next transaction
            end else begin
              input_byte_counter <= input_byte_counter + 1;
            end
          end
        end

        STATE_COMPUTE: begin
          uio_oe_bit0_reg <= 1'b0;
          C00 <= A0 * B0 + A1 * B2;
          C01 <= A0 * B1 + A1 * B3;
          C10 <= A2 * B0 + A3 * B2;
          C11 <= A2 * B1 + A3 * B3;
          state <= STATE_LOAD_UART_DATA;
          output_byte_counter <= 0;  // Start with the first result byte
        end

        STATE_LOAD_UART_DATA: begin
          uio_oe_bit0_reg <= 1'b1;  // Enable UART TX pin output
          case (output_byte_counter)
            2'd0: begin
              uo_out_reg <= C00[7:0];
              uart_tx_data_reg <= C00[7:0];
            end
            2'd1: begin
              uo_out_reg <= C01[7:0];
              uart_tx_data_reg <= C01[7:0];
            end
            2'd2: begin
              uo_out_reg <= C10[7:0];
              uart_tx_data_reg <= C10[7:0];
            end
            2'd3: begin
              uo_out_reg <= C11[7:0];
              uart_tx_data_reg <= C11[7:0];
            end
          endcase
          uart_tx_start_pulse <= 1'b1;  // Start UART transmission for the loaded byte
          state <= STATE_TRIGGER_UART_SEND;
        end

        STATE_TRIGGER_UART_SEND: begin
          uio_oe_bit0_reg <= 1'b1;  // Keep UART TX pin enabled
          // uart_tx_data_reg is now stable with the correct data from the previous state
          uart_tx_start_pulse <= 1'b1;  // Start UART transmission
          state <= STATE_WAIT_TX_COMPLETE;
        end

        STATE_WAIT_TX_COMPLETE: begin
          uio_oe_bit0_reg <= 1'b1;  // Keep UART TX pin enabled
          // uart_tx_start_pulse is already de-asserted by default
          if (!uart_tx_busy) begin  // Current byte transmission finished
            if (output_byte_counter == 2'd3) begin  // All 4 result bytes sent
              state               <= STATE_WAIT_UART_INPUT;
              output_byte_counter <= 0;
              uio_oe_bit0_reg     <= 1'b0;  // Disable UART TX pin
            end else begin
              output_byte_counter <= output_byte_counter + 1;
              state               <= STATE_LOAD_UART_DATA;  // Load and send next byte
            end
          end
        end

        default: state <= STATE_WAIT_UART_INPUT;
      endcase
    end
  end

  // List all unused input signals to prevent warnings
  // uio_in is not used by the DUT's internal logic in this version.
  // ui_in[0] is used for UART RX. ui_in[7:1] are unused.
  wire _unused = &{ena, ui_in[7:1], uio_in};

endmodule
