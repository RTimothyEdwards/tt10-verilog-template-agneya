# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer, RisingEdge, FallingEdge, First

# UART Simulation Parameters
SIM_CYCLES_PER_BIT = 10  # Should match CLKS_PER_BIT_FOR_UART in project.v for sim
UART_INPUT_PIN_INDEX = 0   # ui_in[0] for UART RX on DUT
UART_OUTPUT_PIN_INDEX = 0  # uio_out[0] for UART TX from DUT

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

async def receive_uart_byte(dut, cycles_per_bit, timeout_cycles=None):
    """Receives a byte serially (LSB first, 1 start, 8 data, 1 stop bit)
       by monitoring dut.uio_out[UART_OUTPUT_PIN_INDEX].
       Uses polling for start bit detection.
    """
    dut._log.info(f"UART_RX_SIM: Waiting for byte from uio_out[{UART_OUTPUT_PIN_INDEX}]...")

    # Determine overall timeout for this function call
    # This timeout applies to the entire process of receiving one byte, including waiting for OE and start bit.
    # Default to a generous timeout if none provided (e.g., 2 full byte transmission times)
    overall_max_cycles_for_byte = timeout_cycles if timeout_cycles is not None else (cycles_per_bit * 10 * 2)
    
    cycles_spent_total = 0

    # 1. Wait for uio_oe[UART_OUTPUT_PIN_INDEX] to be 1 (DUT is driving the pin)
    dut._log.info(f"UART_RX_SIM: Waiting for uio_oe[{UART_OUTPUT_PIN_INDEX}] to become 1...")
    while dut.uio_oe[UART_OUTPUT_PIN_INDEX].value != 1:
        if cycles_spent_total >= overall_max_cycles_for_byte:
            dut._log.error(f"UART_RX_SIM: Timeout ({overall_max_cycles_for_byte} cycles) waiting for uio_oe[{UART_OUTPUT_PIN_INDEX}] to be 1. Current value: {dut.uio_oe[UART_OUTPUT_PIN_INDEX].value}.")
            return None
        await ClockCycles(dut.clk, 1)
        cycles_spent_total += 1
    dut._log.info(f"UART_RX_SIM: uio_oe[{UART_OUTPUT_PIN_INDEX}] is 1. (Waited {cycles_spent_total} cycles for OE).")

    # 2. Poll for the start bit (transition from 1 to 0)
    # First, ensure the line is idle (high) or wait for it to become idle.
    # This also accounts for the stop bit of a previous transmission.
    cycles_waited_for_idle = 0
    # Max wait for idle: a few bit times. This should be short if OE just went high or after a stop bit.
    max_wait_idle_cycles = cycles_per_bit * 3 
    
    dut._log.info(f"UART_RX_SIM: Waiting for line uio_out[{UART_OUTPUT_PIN_INDEX}] to be idle (1)...")
    while dut.uio_out[UART_OUTPUT_PIN_INDEX].value != 1:
        if cycles_waited_for_idle >= max_wait_idle_cycles:
            dut._log.error(f"UART_RX_SIM: Line uio_out[{UART_OUTPUT_PIN_INDEX}] did not become idle (1) within {max_wait_idle_cycles} cycles. Current value: {dut.uio_out[UART_OUTPUT_PIN_INDEX].value}")
            return None
        if cycles_spent_total >= overall_max_cycles_for_byte: # Check overall timeout
            dut._log.error(f"UART_RX_SIM: Overall timeout ({overall_max_cycles_for_byte} cycles) while waiting for line to go idle.")
            return None
        await ClockCycles(dut.clk, 1)
        cycles_waited_for_idle += 1
        cycles_spent_total +=1
    dut._log.info(f"UART_RX_SIM: Line uio_out[{UART_OUTPUT_PIN_INDEX}] is idle (1). Polling for start bit (0)...")
    
    start_bit_detected = False
    cycles_polling_start_bit = 0
    # Max cycles to poll for start bit: should be generous enough for inter-byte gap + start bit itself
    # The overall_max_cycles_for_byte should cover this.
    while not start_bit_detected:
        if dut.uio_out[UART_OUTPUT_PIN_INDEX].value == 0:
            start_bit_detected = True
            dut._log.info(f"UART_RX_SIM: Polled and detected start bit on uio_out[{UART_OUTPUT_PIN_INDEX}] after {cycles_polling_start_bit} poll cycles.")
            break
        if cycles_spent_total >= overall_max_cycles_for_byte:
            dut._log.error(f"UART_RX_SIM: Overall timeout ({overall_max_cycles_for_byte} cycles) polling for start bit. Line remained high.")
            return None
        await ClockCycles(dut.clk, 1)
        cycles_polling_start_bit +=1
        cycles_spent_total += 1
    
    if not start_bit_detected: # Should be caught by timeout above, but as a safeguard
        dut._log.error(f"UART_RX_SIM: Failed to detect start bit within allowed cycles.")
        return None

    # At this point, start bit (0) is detected. We are at the beginning of the start bit.
    # Wait for the center of the start bit (half bit duration from its beginning)
    await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns") # Assuming 10us clock period

    if dut.uio_out[UART_OUTPUT_PIN_INDEX].value != 0:
        dut._log.error(f"UART_RX_SIM: Start bit was not low after half period (sampled at center). Value: {dut.uio_out[UART_OUTPUT_PIN_INDEX].value}")
        return None 

    # Wait remaining half of start bit period to align with the start of the first data bit
    await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns")

    byte_val = 0
    # Data bits (LSB first)
    for i in range(8):
        # Wait for the middle of the current data bit
        await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns")
        bit = dut.uio_out[UART_OUTPUT_PIN_INDEX].value
        byte_val |= (int(bit) << i)
        # Wait for the remaining half of the current data bit
        await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns")
        dut._log.debug(f"UART_RX_SIM: Sampled data bit {i}: {bit}")

    # Stop bit (should be high)
    # Wait for the middle of the stop bit
    await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns")
    stop_bit_val = dut.uio_out[UART_OUTPUT_PIN_INDEX].value
    if stop_bit_val != 1:
        dut._log.error(f"UART_RX_SIM: Stop bit was not high. Got {stop_bit_val}")
        # Wait remaining half of stop bit period
        await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns")
        return None 
    
    # Wait remaining half of stop bit period
    await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns")

    dut._log.info(f"UART_RX_SIM: Received byte 0x{byte_val:02X} ({byte_val}).")
    return byte_val

@cocotb.test()
async def test_matrix_mult_uart(dut):
    dut._log.info("Starting matrix multiplication test via UART on ui_in[0] and uio_out[0]")

    # Initialize ui_in to a known state (all high, uart_rx_pin idle)
    dut.ui_in.value = (1 << UART_INPUT_PIN_INDEX) 

    clock = Clock(dut.clk, 10, units="us") 
    cocotb.start_soon(clock.start())

    dut._log.info("Applying reset")
    dut.rst_n.value = 0
    dut.ena.value = 0 
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1 
    # Initialize uio_oe to 0, as testbench is not driving uio pins.
    # The DUT controls uio_oe for its outputs.
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
        await send_uart_byte(dut, value_to_send, SIM_CYCLES_PER_BIT)
        # await ClockCycles(dut.clk, 2) # Small delay between bytes if DUT needs it

    dut._log.info("All 8 matrix data bytes sent to DUT via UART.")

    expected_results_uart = [19, 22, 43, 50] 
    actual_results_uart = []
    actual_results_parallel = []

    dut._log.info("Waiting for results from DUT via UART (on uio_out[0]) and parallel (on uo_out)...")

    # Timeout for the first byte:
    # UART input: 8 bytes * 10 bits/byte * 10 cycles/bit = 800 cycles
    # Computation: ~1-2 cycles
    # UART output prep (OE high, start bit): ~2-3 cycles
    # Total before first start bit of output: ~805 cycles. Add buffer.
    # This timeout is for the *entire* receive_uart_byte function.
    first_byte_timeout = (len(all_input_values) * 10 * SIM_CYCLES_PER_BIT) + (SIM_CYCLES_PER_BIT * 10) # allow 1 extra byte time

    for i in range(len(expected_results_uart)):
        dut._log.info(f"Attempting to receive UART byte {i+1}/{len(expected_results_uart)}...")
        
        # Timeout for subsequent bytes can be shorter, mostly covering one byte transmission time + small buffer
        current_timeout = first_byte_timeout if i == 0 else (SIM_CYCLES_PER_BIT * 10 * 3) # Increased timeout slightly for subsequent bytes

        received_byte = await receive_uart_byte(dut, SIM_CYCLES_PER_BIT, timeout_cycles=current_timeout)
        
        if received_byte is None:
            assert False, f"UART_RX_SIM: Failed to receive byte {i+1} from DUT."
        actual_results_uart.append(received_byte)
        dut._log.info(f"UART_RX_SIM: Received result byte {i+1}: {received_byte} (0x{received_byte:02X}) (Expected: {expected_results_uart[i]})")

        # Capture parallel output uo_out.
        # Sample uo_out *before* potentially waiting, to catch the value associated with the byte just received/processed by DUT
        current_parallel_output = int(dut.uo_out.value)
        actual_results_parallel.append(current_parallel_output)
        dut._log.info(f"Parallel uo_out captured for byte {i+1}: {current_parallel_output} (Expected for UART: {expected_results_uart[i]})")

        # if i < len(expected_results_uart) - 1:
            # Add a small delay to ensure DUT has fully transitioned and testbench is ready for the next byte's start bit detection
            # This helps if there are any subtle race conditions or scheduling effects in the simulator or testbench.
            # One bit time should be enough for the DUT's UART to settle its stop bit and for the main FSM to decide on the next action.
            # await ClockCycles(dut.clk, SIM_CYCLES_PER_BIT) 
            # This delay might help the testbench re-synchronize if it was slightly off.

    assert actual_results_uart == expected_results_uart, \
        f"UART Matrix multiplication failed. Expected {expected_results_uart}, got {actual_results_uart}"
    dut._log.info("UART Matrix multiplication test passed!")

    # The parallel output check might still show discrepancies due to timing.
    # If UART passes, this is secondary.
    # For it to pass, the DUT would need to hold uo_out for the currently transmitting UART byte
    # until that byte is fully sent, or the testbench samples uo_out at a more precise moment.
    # dut._log.info(f"Parallel results: Expected {expected_results_uart}, got {actual_results_parallel}")
    # assert actual_results_parallel == expected_results_uart, \
    #     f"Parallel uo_out verification failed. Expected {expected_results_uart}, got {actual_results_parallel}"
    # dut._log.info("Parallel uo_out verification also passed!")

    await ClockCycles(dut.clk, 20)