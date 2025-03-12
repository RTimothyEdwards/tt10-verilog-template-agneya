`default_nettype none
/*
 *  SPDX-FileCopyrightText: 2015 Clifford Wolf
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  SPDX-License-Identifier: ISC
 */

/* This version of simpleuart taken from picosoc (picorv32) and modified
 * for operation as a module sitting between the FTDI chip on the Arty A7
 * board and the Caravel housekeeping SPI, translating from UART protocol
 * to SPI and vice versa, because the Arty A7 board does not connect pins
 * from the FTDI other than UART Tx and Rx, making a direct connection to
 * SPI impossible.
 *
 * UART format is:  1 start bit (0), 8 data bits, 1 stop bit (1)
 *
 * 2nd pass at this module.  Noting that a 1-byte delay is acceptable
 * since the return data may arrive at any time relative to the outbound
 * data (because the UART is full-duplex and the input and output bits
 * don't have to be in lockstep like they are with only the SPI).  Using
 * the delay to save the contents of a full byte on one side, the bits
 * are flipped between the UART protocol (lsb first) and SPI protocol
 * (msb first).
 *
 * To deal with the CSB line, CSB is assumed to be held low for the
 * duration of a transmission.  Then end of transmission is determined
 * by the lack of additional outgoing data, which is defined as the
 * time span equivalent to N bytes, without another start bit.
 *
 * In the worst case, N may have to be set very high to cover delays in
 * the output, and any driving program will have to add pauses to force
 * the CSB to go high to end a transmission.
 */

module uart_to_spi (
    input wire clk,
    input wire resetn,

    output wire ser_tx,
    input  wire ser_rx,

    input  wire spi_sdo,
    output reg spi_csb,
    output reg spi_sdi,
    output reg spi_sck,

    output wire mgmt_uart_rx,
    input wire mgmt_uart_tx,
    input wire mgmt_uart_enabled
);
    wire [15:0] cfg_divider;	// Fixed divider

    // Registers for receiving data from the FTDI UART
    reg [3:0] recv_state;
    reg [15:0] recv_divcnt;
    reg [7:0] recv_pattern;
    reg [7:0] recv_buf_data;
    reg recv_buf_valid;

    // Registers for sending SPI data to Caravel
    reg [2:0] send_state;
    reg [3:0] send_bitcnt;
    reg [15:0] send_divcnt;
    reg [7:0] send_buf_data;

    // Registers for sending return SPI data back to the FTDI UART
    reg retn_bit;
    reg retn_active;
    reg [1:0] retn_state;
    reg [9:0] retn_pattern;
    reg [3:0] retn_bitcnt;
    reg [15:0] retn_divcnt;

    // Set divider for fixed baud rate (TBD)
    // (Set to 100 for operation at 1Mbps)
    // assign cfg_divider = 16'd100;
    // (Set to 200 for operation at 500kbps)
    // assign cfg_divider = 16'd200;
    // (Set to 1042 for operation at 96kbps to match the other on-chip UART)
    assign cfg_divider = 16'd1042;

    always @(posedge clk) begin
        if (!resetn) begin
            recv_state <= 0;
            recv_divcnt <= 0;
            recv_pattern <= 0;
            recv_buf_data <= 0;
            recv_buf_valid <= 0;
        end else begin
            recv_divcnt <= recv_divcnt + 1;
            case (recv_state)
                0: begin
                    if (!ser_rx) begin
                        recv_state <= 1;
		    end
                    recv_divcnt <= 0;
                    recv_buf_valid <= 0;
                end
                1: begin
		    // Wait for 1/2 expected bit period so that the
		    // input will be sampled approximately in the middle.

                    if (2*recv_divcnt > cfg_divider) begin
                        recv_state <= 2;
                        recv_divcnt <= 0;
                    end
                end
                10: begin
                    if (recv_divcnt > cfg_divider) begin
                        recv_buf_data <= recv_pattern;
                        recv_buf_valid <= 1;
                        recv_state <= 11;
			recv_divcnt <= 0;
                    end
                end
                11: begin
                    recv_divcnt <= 0;
		    recv_state <= 0;
                end
                default: begin
		    if (recv_divcnt > cfg_divider) begin
                        recv_pattern <= {ser_rx, recv_pattern[7:1]};
                        recv_state <= recv_state + 1;
                        recv_divcnt <= 0;
                    end
                end
            endcase
        end
    end

    // Multiplex the UART transmit with the one from Caravel.
    // If the UART is turned on in software then the housekeeping SPI
    // will continue to communicate, but care must be taken not to
    // let the two UARTs interfere with each other.

    assign ser_tx = mgmt_uart_enabled ? (mgmt_uart_tx & retn_pattern[0]) :
    		retn_pattern[0];

    // UART receive channel from FTDI is copied to both on-chip UARTs.
    assign mgmt_uart_rx = ser_rx;

    always @(posedge clk) begin
        if (!resetn) begin
	    send_divcnt <= 0;
	    send_state <= 0;
	    send_bitcnt <= 0;
	    send_buf_data <= 0;
	    retn_bit <= 0;
	    spi_csb <= 1;
	    spi_sck <= 0;
	    spi_sdi <= 0;
	end else begin
	    case (send_state)
		0: begin
		    if (recv_buf_valid == 1) begin
		        spi_csb <= 0;
		        send_state <= 1;
			send_bitcnt <= 0;
			send_buf_data <= recv_buf_data;
			send_divcnt <= 0;
		    end else if (recv_state != 0) begin
			send_divcnt <= 0;
		    end else if (send_divcnt > 16*cfg_divider) begin
			// Timeout, end stream by raising CSB.
			spi_csb <= 1;
		    end else begin
			send_divcnt <= send_divcnt + 1;
		    end
		end
		1: begin
		    spi_sck <= 0;
		    spi_sdi <= send_buf_data[7];
		    send_divcnt <= 0;
		    send_state <= 2;
		end
		2: begin
		    send_divcnt <= send_divcnt + 1;
		    if (2*send_divcnt > cfg_divider) begin
		        send_buf_data <= {send_buf_data[6:0], 1'b0};
		        spi_sck <= 1;
			retn_bit <= spi_sdo;
			send_divcnt <= 0;
		        send_bitcnt <= send_bitcnt + 1;
			send_state <= 3;
		    end
		end
		3: begin
		    send_divcnt <= send_divcnt + 1;
		    if (2*send_divcnt > cfg_divider) begin
			spi_sck <= 0;
		        send_buf_data[0] <= retn_bit;
			spi_sdi <= send_buf_data[7];
			send_divcnt <= 0;
			if (send_bitcnt == 8) begin
			    send_state <= 4;
			end else begin
			    send_state <= 2;
			end
		    end
		end
		4: begin
		    send_state <= 0;
		end
	    endcase
	end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            retn_pattern <= ~0;
            retn_bitcnt <= 0;
            retn_divcnt <= 0;
	    retn_state <= 0;
	    retn_active <= 0;
        end else begin
	    retn_divcnt <= retn_divcnt + 1;
	    if (send_state == 4) begin
		retn_active <= 1;
	    end
	    case (retn_state)
		0: begin
		    if (retn_active == 1) begin
		        retn_pattern <= {1'b1, send_buf_data, 1'b0};
		        retn_bitcnt <= 10;
		        retn_divcnt <= 0;
		        retn_state <= 1;
			retn_active <= 0;
		    end
		end
		1: begin
		    if (retn_divcnt > cfg_divider && retn_bitcnt) begin
			retn_pattern <= {1'b1, retn_pattern[9:1]};
			retn_bitcnt <= retn_bitcnt - 1;
			retn_divcnt <= 0;
		    end else if (!retn_bitcnt) begin
			retn_state <= 0;
		    end
		end
            endcase
        end
    end
endmodule
`default_nettype wire