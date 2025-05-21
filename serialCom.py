import serial
import time

# Configure your serial port (adjust COM port as needed)
# On Windows: 'COMx' (e.g., 'COM3')
# On Linux: '/dev/ttyUSBx' (e.g., '/dev/ttyUSB0')
# On macOS: '/dev/cu.usbserial-xxxx' or '/dev/tty.usbserial-xxxx'
SERIAL_PORT = 'COM10' # CHANGE THIS
BAUD_RATE = 115200   # Must match your FPGA design

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) # 1-second timeout
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

    # Example: Sending the same matrix data as your testbench
    matrix_A_values = [1, 2, 3, 4]
    matrix_B_values = [5, 6, 7, 8]
    all_input_values = matrix_A_values + matrix_B_values

    for val in all_input_values:
        print(f"Sending: {val}")
        ser.write(bytes([val])) # Send one byte
        time.sleep(0.01) # Small delay, might not be strictly necessary but can help

    print("All input bytes sent. Waiting for results...")

    expected_results_count = 4
    received_results = []
    for i in range(expected_results_count):
        # Read one byte, with timeout
        received_byte = ser.read(1)
        if received_byte:
            val = int.from_bytes(received_byte, 'big')
            received_results.append(val)
            print(f"Received byte {i+1}: {val} (0x{val:02X})")
        else:
            print(f"Timeout waiting for byte {i+1}")
            break
    
    print(f"Expected: [19, 22, 43, 50], Got: {received_results}")

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")