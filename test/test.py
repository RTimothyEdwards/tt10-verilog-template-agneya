# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer, RisingEdge

# UART Simulation Parameters
# These should match the DUT's UART configuration for the simulation
# Clock is 100kHz (10us period)
# Baud rate is 10k
# Therefore, cycles per bit = 100kHz / 10kBaud = 10
SIM_CYCLES_PER_BIT = 10
UART_INPUT_PIN_INDEX = 0 # We'll use ui_in[0] for UART RX on DUT

async def send_uart_byte(dut, byte_val, cycles_per_bit):
    """Sends a byte serially (LSB first, 1 start, 8 data, 1 stop bit)
       by wiggling dut.ui_in[UART_INPUT_PIN_INDEX].
    """
    dut._log.info(f"UART_TX_SIM: Sending byte 0x{byte_val:02X} ({byte_val}) to ui_in[{UART_INPUT_PIN_INDEX}]")
    
    # Start bit (low)
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 0
    await ClockCycles(dut.clk, cycles_per_bit)

    # Data bits (LSB first)
    for i in range(8):
        bit = (byte_val >> i) & 1
        dut.ui_in[UART_INPUT_PIN_INDEX].value = bit
        await ClockCycles(dut.clk, cycles_per_bit)

    # Stop bit (high)
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1
    await ClockCycles(dut.clk, cycles_per_bit)
    dut._log.info(f"UART_TX_SIM: Byte 0x{byte_val:02X} sent.")

@cocotb.test()
async def test_matrix_mult_uart(dut):
    dut._log.info("Starting matrix multiplication test via UART on ui_in[0]")

    # Initialize ui_in to a known state (all high, uart_rx_pin idle)
    dut.ui_in.value = (1 << UART_INPUT_PIN_INDEX) # Set UART pin to idle (high), rest don't care or 0

    # Clock with 10us period (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Applying reset")
    dut.rst_n.value = 0
    dut.ena.value = 0 # Keep ena low during reset
    # Ensure UART line is idle during reset
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1 
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    dut.ena.value = 1 # Assert ena after reset
    # Keep UART line idle after reset
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1 
    await ClockCycles(dut.clk, 5) 

    matrix_A_values = [1, 2, 3, 4]
    matrix_B_values = [5, 6, 7, 8]
    all_input_values = matrix_A_values + matrix_B_values

    dut._log.info("Sending matrix data via UART...")
    for value_to_send in all_input_values:
        await send_uart_byte(dut, value_to_send, SIM_CYCLES_PER_BIT)
        # Optional small delay between bytes if needed, though the FSM should handle back-to-back
        # await ClockCycles(dut.clk, 1) 

    dut._log.info("All 8 matrix data bytes sent via UART.")

    expected_results = [19, 22, 43, 50] # C00, C01, C10, C11 (truncated)
    actual_results = []

    # Wait for computation and output.
    # UART sending: 8 bytes * (1 start + 8 data + 1 stop bits) * SIM_CYCLES_PER_BIT/bit
    # = 8 * 10 bits/byte * 10 cycles/bit = 800 clock cycles.
    # Add some buffer for DUT FSM to process last byte and start computation/output.
    dut._log.info("Waiting for computation and first output value...")
    # The DUT FSM takes 1 cycle to recognize byte_ready, 1 for compute, 1 for first output.
    # So, after the last bit of the stop bit is sent, wait a few cycles.
    # send_uart_byte already includes the stop bit duration.
    await ClockCycles(dut.clk, 2) # Wait a bit for FSM to transition and output first result

    for i in range(len(expected_results)):
        # For the first result, it might be ready. For subsequent, wait for next clock.
        # if i > 0:
        #      await RisingEdge(dut.clk) # Ensure we sample after potential clock edge update
        # Or simply await ClockCycles(dut.clk, 1) if not the first item.
        # await ClockCycles(dut.clk, 1) # Wait for next output from FSM if not the first one

        current_output = int(dut.uo_out.value)
        actual_results.append(current_output)
        dut._log.info(f"Output {i}: {current_output} (Expected: {expected_results[i]})")
        if i < len(expected_results) -1 : # Avoid waiting after the last result is captured
            await ClockCycles(dut.clk, 1) # Wait for the FSM to prepare the next output

    assert actual_results == expected_results, \
        f"Matrix multiplication failed. Expected {expected_results}, got {actual_results}"

    dut._log.info("Matrix multiplication test (UART) passed!")
    await ClockCycles(dut.clk, 20)