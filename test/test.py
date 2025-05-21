# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer, RisingEdge, FallingEdge, First


CLOCK_FREQ_HZ = 100_000_000  # 100 MHz
BAUD_RATE = 115200

# UART Simulation Parameters
CYCLES_PER_BIT = int(CLOCK_FREQ_HZ // BAUD_RATE)  # Should match CLKS_PER_BIT_FOR_UART in project.v for sim
UART_INPUT_PIN_INDEX = 0   # ui_in[0] for UART RX on DUT
UART_OUTPUT_PIN_INDEX = 0  # uio_out[0] for UART TX from DUT

async def send_uart_byte(dut, byte_val):
    # Start bit
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 0
    await ClockCycles(dut.clk, CYCLES_PER_BIT)
    # Data bits
    for i in range(8):
        dut.ui_in[UART_INPUT_PIN_INDEX].value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CYCLES_PER_BIT)
    # Stop bit
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1
    await ClockCycles(dut.clk, CYCLES_PER_BIT)

async def receive_uart_byte(dut):
    # Wait for start bit
    while dut.uio_out[UART_OUTPUT_PIN_INDEX].value != 0:
        await ClockCycles(dut.clk, 1)
    # Wait half a bit to sample in the middle
    await ClockCycles(dut.clk, CYCLES_PER_BIT // 2)
    # Sample start bit (should be 0)
    if dut.uio_out[UART_OUTPUT_PIN_INDEX].value != 0:
        return None
    # Sample data bits
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CYCLES_PER_BIT)
        bit = dut.uio_out[UART_OUTPUT_PIN_INDEX].value
        byte_val |= (int(bit) << i)
    # Sample stop bit
    await ClockCycles(dut.clk, CYCLES_PER_BIT)
    return byte_val

@cocotb.test()
async def test_matrix_mult_uart(dut):
    dut._log.info("Starting matrix multiplication test via UART on ui_in[0] and uio_out[0]")

    # Initialize ui_in to a known state (all high, uart_rx_pin idle)
    dut.ui_in.value = (1 << UART_INPUT_PIN_INDEX)

    clock = Clock(dut.clk, 1e9 / CLOCK_FREQ_HZ, units="ns")  # 10 ns for 100 MHz
    cocotb.start_soon(clock.start())

    dut._log.info("Applying reset")
    dut.rst_n.value = 0
    dut.ena.value = 0 
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1 
    dut.uio_oe.value = 0 
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    dut.ena.value = 1 
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1 
    await ClockCycles(dut.clk, 5) 

    matrix_A_values = [1, 2, 3, 4]
    matrix_B_values = [5, 6, 7, 8]
    all_input_values = matrix_A_values + matrix_B_values

    dut._log.info("Sending matrix data via UART to DUT...")
    for value_to_send in all_input_values:
        await send_uart_byte(dut, value_to_send)
        # Optionally, add a small delay if needed:
        # await ClockCycles(dut.clk, CYCLES_PER_BIT // 2)

    dut._log.info("All 8 matrix data bytes sent to DUT via UART.")

    expected_results_uart = [19, 22, 43, 50] 
    actual_results_uart = []
    actual_results_parallel = []

    dut._log.info("Waiting for results from DUT via UART (on uio_out[0]) and parallel (on uo_out)...")

    # Timeout for the first byte: allow enough cycles for all input, computation, and output
    # 8 bytes in, 4 bytes out, each byte is 10 bits (start+8+stop), each bit is CYCLES_PER_BIT
    first_byte_timeout = (len(all_input_values) * 10 * CYCLES_PER_BIT) + (CYCLES_PER_BIT * 20)

    for i in range(len(expected_results_uart)):
        dut._log.info(f"Attempting to receive UART byte {i+1}/{len(expected_results_uart)}...")

        # Wait for the next result byte with a generous timeout
        received_byte = None
        cycles_waited = 0
        while received_byte is None and cycles_waited < first_byte_timeout:
            received_byte = await receive_uart_byte(dut)
            cycles_waited += (10 * CYCLES_PER_BIT)  # Each receive_uart_byte call waits at least one byte time

        assert received_byte is not None, f"UART_RX_SIM: Failed to receive byte {i+1} from DUT."
        actual_results_uart.append(received_byte)
        dut._log.info(f"UART_RX_SIM: Received result byte {i+1}: {received_byte} (0x{received_byte:02X}) (Expected: {expected_results_uart[i]})")

        # Capture parallel output uo_out.
        current_parallel_output = int(dut.uo_out.value)
        actual_results_parallel.append(current_parallel_output)
        dut._log.info(f"Parallel uo_out captured for byte {i+1}: {current_parallel_output} (Expected for UART: {expected_results_uart[i]})")

    assert actual_results_uart == expected_results_uart, \
        f"UART Matrix multiplication failed. Expected {expected_results_uart}, got {actual_results_uart}"
    dut._log.info("UART Matrix multiplication test passed!")

    # Optionally check parallel output
    # dut._log.info(f"Parallel results: Expected {expected_results_uart}, got {actual_results_parallel}")
    # assert actual_results_parallel == expected_results_uart, \
    #     f"Parallel uo_out verification failed. Expected {expected_results_uart}, got {actual_results_parallel}"
    # dut._log.info("Parallel uo_out verification also passed!")

    await ClockCycles(dut.clk, 20)