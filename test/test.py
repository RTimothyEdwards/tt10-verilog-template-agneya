# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

@cocotb.test()
async def test_matrix_mult(dut):
    dut._log.info("Starting matrix multiplication test")

    # clock with 10us period (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # reset
    dut._log.info("Applying reset")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    # await ClockCycles(dut.clk, 2)

    # input A via ui_in [1, 2, 3, 4]
    for value in [1, 2, 3, 4]:
        dut.ui_in.value = value
        await ClockCycles(dut.clk, 1)
    dut._log.info("Matrix A loaded")

    # input B via uio_in [5, 6, 7, 8]
    for value in [5, 6, 7, 8]:
        dut.uio_in.value = value
        await ClockCycles(dut.clk, 1)
    dut._log.info("Matrix B loaded")

    # Expected outputs: 19, 22, 43, 50 (lower 8 bits of each sum)
    expected = [19, 22, 43, 50]
    results = []

    # wait for computation or something
    await ClockCycles(dut.clk, 2)

    for i in range(4):
        await ClockCycles(dut.clk, 1)
        result = int(dut.uo_out.value)
        results.append(result)
        dut._log.info(f"Cycle {i} output: {result}")

    assert results == expected, f"Matrix multiplication failed. Expected {expected}, got {results}"

    dut._log.info("Matrix multiplication test passed!")