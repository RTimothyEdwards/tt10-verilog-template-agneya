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
    dut._log.info(f"UART_RX_SIM: Waiting for byte from uio_out[{UART_OUTPUT_PIN_INDEX}] (polling for start bit)...")

    # Determine timeout for start bit detection
    # If timeout_cycles is provided, use it. Otherwise, a default based on bit time.
    # Total bits for one UART frame = 1 (start) + 8 (data) + 1 (stop) = 10 bits.
    # Allow a bit more than one full byte time for the start bit to appear.
    actual_timeout_cycles = timeout_cycles if timeout_cycles is not None else (cycles_per_bit * 12) 
    
    # Ensure uio_oe[0] is high (DUT is driving)
    # This check can be useful for debugging, but the primary issue is FallingEdge setup
    if dut.uio_oe[UART_OUTPUT_PIN_INDEX].value != 1:
        dut._log.warning(f"UART_RX_SIM: uio_oe[{UART_OUTPUT_PIN_INDEX}] is not 1. Current value: {dut.uio_oe[UART_OUTPUT_PIN_INDEX].value}. Proceeding anyway.")
        # In a real scenario, you might wait for OE to be high or error out.

    # Poll for the start bit (transition from 1 to 0)
    # First, ensure the line is idle (high) or wait for it to become idle
    cycles_waited_for_idle = 0
    # Wait for up to a few bit times for line to go idle if it's not already
    while dut.uio_out[UART_OUTPUT_PIN_INDEX].value != 1 and cycles_waited_for_idle < (cycles_per_bit * 3):
        await ClockCycles(dut.clk, 1)
        cycles_waited_for_idle += 1
    
    if dut.uio_out[UART_OUTPUT_PIN_INDEX].value != 1:
        dut._log.error(f"UART_RX_SIM: Line uio_out[{UART_OUTPUT_PIN_INDEX}] did not become idle (1) before expecting start bit. Current value: {dut.uio_out[UART_OUTPUT_PIN_INDEX].value}")
        return None

    dut._log.info(f"UART_RX_SIM: Line uio_out[{UART_OUTPUT_PIN_INDEX}] is idle (1). Polling for start bit (0)...")
    
    start_bit_detected = False
    for _ in range(actual_timeout_cycles):
        if dut.uio_out[UART_OUTPUT_PIN_INDEX].value == 0:
            start_bit_detected = True
            dut._log.info(f"UART_RX_SIM: Polled and detected start bit on uio_out[{UART_OUTPUT_PIN_INDEX}].")
            break
        await ClockCycles(dut.clk, 1)
    
    if not start_bit_detected:
        dut._log.error(f"UART_RX_SIM: Timeout polling for start bit on uio_out[{UART_OUTPUT_PIN_INDEX}]. Line remained high.")
        return None

    # At this point, start bit (0) is detected.
    # Wait for half a bit period from the detected falling edge to sample in the middle of the start bit
    # The previous loop iteration consumed one clock cycle where the bit was detected as 0.
    # So, we need to wait (cycles_per_bit / 2) - 1 more clock cycles if sampling at center.
    # Or, more simply, proceed to sample based on full bit timings from the detected edge.
    # The current logic uses Timer which is absolute time, so we need to align.
    # Since we just detected the '0', we are at the beginning of the start bit.
    
    # Wait for the center of the start bit (half bit duration from its beginning)
    await Timer( (cycles_per_bit / 2.0) * (10 * 1000), units="ns") # 10us clock period from test

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
    # uio_in is not actively driven by this test for data input to DUT.
    # uio_out will be driven by DUT.

    # Clock with 10us period (100 KHz)
    clock = Clock(dut.clk, 10, units="us") # This matches SIM_CLOCK_FREQ_HZ = 100_000
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Applying reset")
    dut.rst_n.value = 0
    dut.ena.value = 0 
    dut.ui_in[UART_INPUT_PIN_INDEX].value = 1 
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

    expected_results_uart = [19, 22, 43, 50] # C00, C01, C10, C11 (truncated)
    actual_results_uart = []
    
    # Also capture parallel uo_out for comparison/completeness if desired
    actual_results_parallel = []

    dut._log.info("Waiting for results from DUT via UART (on uio_out[0]) and parallel (on uo_out)...")

    # Timeout for the first byte:
    # UART input: 8 bytes * 10 bits/byte * 10 cycles/bit = 800 cycles
    # Computation: ~1-2 cycles
    # UART output prep: ~1-2 cycles
    # Total before first start bit of output: ~805 cycles. Add buffer.
    uart_receive_timeout = (len(all_input_values) * 10 * SIM_CYCLES_PER_BIT) + 100 

    for i in range(len(expected_results_uart)):
        dut._log.info(f"Attempting to receive UART byte {i+1}...")
        # The uo_out (parallel) should update around the time UART TX starts for that byte
        # We can sample uo_out before or after receiving the UART byte.
        # Let's try to sample it just before we expect the UART byte to be fully received.
        
        # If the DUT updates uo_out and starts UART TX in the same state,
        # uo_out might be valid slightly before the UART byte is fully through.
        # For simplicity, we'll focus on UART receive first.

        received_byte = await receive_uart_byte(dut, SIM_CYCLES_PER_BIT, timeout_cycles=uart_receive_timeout if i == 0 else (10 * SIM_CYCLES_PER_BIT + 50) )
        if received_byte is None:
            assert False, f"UART_RX_SIM: Failed to receive byte {i+1} from DUT."
        actual_results_uart.append(received_byte)
        dut._log.info(f"UART_RX_SIM: Received result byte {i+1}: {received_byte} (Expected: {expected_results_uart[i]})")

        # Capture parallel output uo_out. It should correspond to the byte just sent/being sent.
        # This requires careful timing alignment with DUT's FSM.
        # The DUT's STATE_LOAD_AND_SEND_UART updates uo_out_reg and starts uart_tx.
        # So uo_out should be valid while that byte is being transmitted.
        # We can sample it after receive_uart_byte completes for that byte.
        current_parallel_output = int(dut.uo_out.value)
        actual_results_parallel.append(current_parallel_output)
        dut._log.info(f"Parallel uo_out captured for byte {i+1}: {current_parallel_output} (Expected: {expected_results_uart[i]})")


    assert actual_results_uart == expected_results_uart, \
        f"UART Matrix multiplication failed. Expected {expected_results_uart}, got {actual_results_uart}"
    dut._log.info("UART Matrix multiplication test passed!")

    # Also check parallel results if desired
    assert actual_results_parallel == expected_results_uart, \
        f"Parallel uo_out verification failed. Expected {expected_results_uart}, got {actual_results_parallel}"
    dut._log.info("Parallel uo_out verification also passed!")


    await ClockCycles(dut.clk, 20)