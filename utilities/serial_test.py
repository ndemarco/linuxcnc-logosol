#!/usr/bin/env python3
"""
Simple serial port test to verify basic communication
"""

import serial
import time
import sys

def test_serial_port(port='/dev/ttyUSB0', baud=19200):
    """Test if we can open and read/write to the serial port"""
    print(f"Testing {port} at {baud} baud...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        print(f"✓ Port opened successfully")
        print(f"  Port: {ser.port}")
        print(f"  Baud: {ser.baudrate}")
        print(f"  Is open: {ser.is_open}")

        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Try different RTS/DTR settings for RS-485 direction control
        print(f"\nTrying RTS/DTR combinations for RS-485...")

        # Common RS-485 configurations:
        # Option 1: RTS high = transmit, RTS low = receive
        # Option 2: DTR for direction control

        for rts_state in [True, False]:
            for dtr_state in [True, False]:
                ser.rts = rts_state
                ser.dtr = dtr_state
                print(f"\n  Testing RTS={rts_state}, DTR={dtr_state}")

                ser.reset_input_buffer()

                # Send HARD_RESET command
                # Format: [0xAA][addr=0xFF][cmd=0x0F][checksum=0x0E]
                reset_cmd = bytes([0xAA, 0xFF, 0x0F, 0x0E])
                bytes_written = ser.write(reset_cmd)

                # Wait for transmission
                ser.flush()
                time.sleep(0.2)

                # Check for response
                waiting = ser.in_waiting
                if waiting > 0:
                    response = ser.read(waiting)
                    print(f"    ✓ FOUND IT! Received {waiting} bytes: {response.hex()}")
                    print(f"    ✓ Working configuration: RTS={rts_state}, DTR={dtr_state}")
                    return True

        print(f"\n✗ No response with any RTS/DTR combination")

        # One more attempt with manual RTS toggling
        print(f"\nTrying manual RTS toggling...")
        ser.reset_input_buffer()

        # Enable transmit (RTS high)
        ser.rts = True
        time.sleep(0.01)

        # Send HARD_RESET command
        reset_cmd = bytes([0xAA, 0xFF, 0x0F, 0x0E])
        bytes_written = ser.write(reset_cmd)
        ser.flush()

        # Switch to receive (RTS low)
        ser.rts = False
        time.sleep(0.1)

        # Check for any response
        waiting = ser.in_waiting
        print(f"Bytes waiting: {waiting}")

        if waiting > 0:
            response = ser.read(waiting)
            print(f"✓ Received: {response.hex()}")
        else:
            print(f"✗ No response received")

        # Try sending a status request to address 1
        print(f"\nSending status request to address 1...")
        # Format: [0xAA][addr=0x01][cmd=0x00][checksum=0x01]
        status_cmd = bytes([0xAA, 0x01, 0x00, 0x01])
        print(f"Sending: {status_cmd.hex()}")
        bytes_written = ser.write(status_cmd)
        print(f"✓ Wrote {bytes_written} bytes")

        ser.flush()
        time.sleep(0.1)

        waiting = ser.in_waiting
        print(f"Bytes waiting: {waiting}")

        if waiting > 0:
            response = ser.read(waiting)
            print(f"✓ Received: {response.hex()}")
        else:
            print(f"✗ No response received")

        ser.close()
        print(f"\n✓ Port closed")
        return True

    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

    print("=" * 50)
    print("LDCN Serial Port Test")
    print("=" * 50)
    print()

    # Test at common LDCN baud rates
    for baud in [19200, 125000, 115200, 57600]:
        print(f"\n{'=' * 50}")
        test_serial_port(port, baud)
        time.sleep(0.5)

    print(f"\n{'=' * 50}")
    print("Test complete")
