`default_nettype none 
`timescale 1ns / 1ps

module uart_receiver_custom #(
    parameter CLKS_PER_BIT = 10  // Number of system clock cycles per UART bit
) (
    input  wire       clk,           // System clock
    input  wire       rst_n,         // Active-low reset
    input  wire       rx_serial_in,  // Serial data input
    output reg  [7:0] data_out,      // Parallel data output
    output reg        byte_ready     // Pulsed high for one clock cycle when a byte is ready
);

  localparam STATE_IDLE = 3'd0;
  localparam STATE_START_BIT = 3'd1;
  localparam STATE_RX_DATA_BITS = 3'd2;
  localparam STATE_STOP_BIT = 3'd3;
  localparam STATE_CLEANUP = 3'd4;

  reg [                     2:0] current_state;
  reg [$clog2(CLKS_PER_BIT)-1:0] clk_counter;  // Counter for clocks within a bit period
  reg [                     2:0] bit_counter;  // Counts received data bits (0 to 7)
  reg [                     7:0] rx_buffer;  // Holds bits as they are received (LSB first)

  // Synchronize rx_serial_in to mitigate metastability
  reg rx_serial_s1, rx_serial_s2;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_serial_s1 <= 1'b1;
      rx_serial_s2 <= 1'b1;
    end else begin
      rx_serial_s1 <= rx_serial_in;
      rx_serial_s2 <= rx_serial_s1;
    end
  end
  wire rx_in_synced = rx_serial_s2;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      current_state <= STATE_IDLE;
      clk_counter   <= 0;
      bit_counter   <= 0;
      rx_buffer     <= 0;
      data_out      <= 0;
      byte_ready    <= 1'b0;
    end else begin
      if (current_state == STATE_CLEANUP) begin  // Ensure byte_ready is a pulse
        byte_ready <= 1'b0;
      end

      case (current_state)
        STATE_IDLE: begin
          byte_ready <= 1'b0;
          if (!rx_in_synced) begin  // Detected falling edge (start bit)
            current_state <= STATE_START_BIT;
            clk_counter   <= 0;
            bit_counter   <= 0;
          end
        end

        STATE_START_BIT: begin
          // Wait for half a bit time then sample center of start bit
          if (clk_counter == ($clog2(CLKS_PER_BIT))'((CLKS_PER_BIT / 2) - 1)) begin
            if (!rx_in_synced) begin  // Start bit is valid (still low)
              current_state <= STATE_RX_DATA_BITS;
              clk_counter   <= 0;  // Reset counter for data bit timing
            end else begin  // False start (glitch)
              current_state <= STATE_IDLE;
            end
          end else begin
            clk_counter <= clk_counter + 1;
          end
        end

        STATE_RX_DATA_BITS: begin
          // Sample in the middle of the bit period
          if (clk_counter == ($clog2(CLKS_PER_BIT))'((CLKS_PER_BIT / 2) - 1)) begin
            rx_buffer[bit_counter] <= rx_in_synced;  // Store LSB first
          end

          if (clk_counter == ($clog2(CLKS_PER_BIT))'(CLKS_PER_BIT - 1)) begin
            clk_counter <= 0;  // Reset for next bit or state change
            if (bit_counter == 3'd7) begin  // All 8 data bits received
              current_state <= STATE_STOP_BIT;
            end else begin
              bit_counter <= bit_counter + 1;
            end
          end else begin
            clk_counter <= clk_counter + 1;
          end
        end

        STATE_STOP_BIT: begin
          // Wait for one full bit period for the stop bit
          if (clk_counter == ($clog2(CLKS_PER_BIT))'(CLKS_PER_BIT - 1)) begin
            if (rx_in_synced) begin  // Stop bit should be high
              data_out      <= rx_buffer;
              byte_ready    <= 1'b1;
              current_state <= STATE_CLEANUP;
            end else begin  // Framing error
              current_state <= STATE_IDLE;  // Or an error state
            end
            clk_counter <= 0;
          end else begin
            clk_counter <= clk_counter + 1;
          end
        end

        STATE_CLEANUP: begin
          byte_ready    <= 1'b0;
          current_state <= STATE_IDLE;
        end

        default: current_state <= STATE_IDLE;
      endcase
    end
  end
endmodule
