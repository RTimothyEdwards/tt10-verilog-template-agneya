`default_nettype none `timescale 1ns / 1ps

module uart_transmitter_custom #(
    parameter CLKS_PER_BIT = 10  // Number of system clock cycles per UART bit
) (
    input  wire       clk,            // System clock
    input  wire       rst_n,          // Active-low reset
    input  wire       tx_start,       // Pulse high for one clock cycle to start transmission
    input  wire [7:0] data_in,        // 8-bit data to transmit
    output wire       tx_serial_out,  // Serial data output
    output reg        tx_busy         // High during transmission
);

  localparam STATE_IDLE = 2'd0;
  localparam STATE_START_BIT = 2'd1;
  localparam STATE_TX_DATA_BITS = 2'd2;
  localparam STATE_STOP_BIT = 2'd3;

  reg [                     1:0] current_state;
  reg [$clog2(CLKS_PER_BIT)-1:0] clk_counter;  // Counter for clocks within a bit period
  reg [                     2:0] bit_counter;  // Counts transmitted data bits (0 to 7)
  reg [                     7:0] tx_buffer;  // Holds the byte to be transmitted
  reg                            tx_serial_out_reg;

  assign tx_serial_out = tx_serial_out_reg;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      current_state     <= STATE_IDLE;
      clk_counter       <= 0;
      bit_counter       <= 0;
      tx_buffer         <= 0;
      tx_serial_out_reg <= 1'b1;  // Idle line is high
      tx_busy           <= 1'b0;
    end else begin
      case (current_state)
        STATE_IDLE: begin
          tx_serial_out_reg <= 1'b1;  // Keep line high when idle
          tx_busy           <= 1'b0;
          if (tx_start) begin
            tx_buffer         <= data_in;
            current_state     <= STATE_START_BIT;
            clk_counter       <= 0;
            bit_counter       <= 0;
            tx_busy           <= 1'b1;
            tx_serial_out_reg <= 1'b0;  // Start bit
          end
        end

        STATE_START_BIT: begin
          tx_serial_out_reg <= 1'b0;  // Transmit Start bit (low)
          if (clk_counter == CLKS_PER_BIT - 1) begin
            clk_counter <= 0;
            current_state <= STATE_TX_DATA_BITS;
            // Prepare first data bit (LSB)
            tx_serial_out_reg <= tx_buffer[0];
          end else begin
            clk_counter <= clk_counter + 1;
          end
        end

        STATE_TX_DATA_BITS: begin
          // Current bit is already on tx_serial_out_reg from previous state or previous cycle
          if (clk_counter == CLKS_PER_BIT - 1) begin
            clk_counter <= 0;
            if (bit_counter == 3'd7) begin  // All 8 data bits sent
              current_state <= STATE_STOP_BIT;
              tx_serial_out_reg <= 1'b1;  // Stop bit
            end else begin
              bit_counter <= bit_counter + 1;
              tx_serial_out_reg <= tx_buffer[bit_counter+1];  // Next bit
            end
          end else begin
            clk_counter <= clk_counter + 1;
          end
        end

        STATE_STOP_BIT: begin
          tx_serial_out_reg <= 1'b1;  // Transmit Stop bit (high)
          if (clk_counter == CLKS_PER_BIT - 1) begin
            clk_counter   <= 0;
            current_state <= STATE_IDLE;
            // tx_busy will be set to 0 in IDLE state on the next cycle
          end else begin
            clk_counter <= clk_counter + 1;
          end
        end

        default: current_state <= STATE_IDLE;
      endcase
    end
  end
endmodule
