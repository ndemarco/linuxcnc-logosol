#!/usr/bin/env python3
"""
Test sending position commands to initialized servos
to see if they respond correctly
"""
import serial
import time
import struct

def send_command(ser, address, command, data_bytes=[]):
    """Send LDCN command and return response"""
    header = 0xAA
    num_data = len(data_bytes)
    cmd_byte = (num_data << 4) | (command & 0x0F)
    checksum = (address + cmd_byte + sum(data_bytes)) & 0xFF
    packet = bytes([header, address, cmd_byte] + data_bytes + [checksum])
    ser.write(packet)
    ser.flush()
    time.sleep(0.02)
    return ser.read(50)

def read_drive_position(ser, addr):
    """Read current position from a drive"""
    status_bits = 0x01 | 0x04 | 0x08 | 0x40  # pos, vel, aux, pos_err
    response = send_command(ser, addr, 0x03, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    if len(response) < 6:
        return None

    status_byte = response[0]
    position = struct.unpack('<i', bytes(response[1:5]))[0]

    return {
        'status': status_byte,
        'position': position,
        'servo_on': bool(status_byte & 0x08),  # Assuming aux byte follows
    }

def send_position_command(ser, addr, position_counts):
    """Send a position command to a drive"""
    # Build trajectory: position, velocity, acceleration
    traj_ctrl = 0x80 | 0x10  # start_now=1, servo_mode=1

    # Pack: control, position (4 bytes), velocity (4 bytes), accel (4 bytes)
    pos_bytes = struct.pack('<i', position_counts)
    vel_bytes = struct.pack('<I', 0)  # velocity = 0
    accel_bytes = struct.pack('<I', 100)  # small acceleration

    traj_data = [traj_ctrl] + list(pos_bytes) + list(vel_bytes) + list(accel_bytes)

    response = send_command(ser, addr, 0x04, traj_data)
    return len(response) > 0

def main():
    PORT = '/dev/ttyUSB0'
    BAUD = 125000
    TEST_ADDR = 1  # Test on drive 1 (Y axis)
    SCALE = 2000.0  # counts/mm

    print("="*70)
    print("POSITION COMMAND TEST")
    print("Tests if drives respond to position commands after initialization")
    print("="*70)

    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.2)

    # Read initial position
    print(f"\n=== INITIAL POSITION (Drive {TEST_ADDR}) ===")
    status = read_drive_position(ser, TEST_ADDR)
    if status:
        print(f"  Status: 0x{status['status']:02X}")
        print(f"  Position (counts): {status['position']}")
        print(f"  Position (mm): {status['position'] / SCALE:.4f}")
        print(f"  Servo on: {status['servo_on']}")
    else:
        print("  Failed to read position")
        ser.close()
        return

    initial_pos = status['position']

    # Send a small position command (move 1mm = 2000 counts)
    target_mm = 1.0
    target_counts = int(target_mm * SCALE)

    print(f"\n=== SENDING POSITION COMMAND ===")
    print(f"  Target: {target_mm} mm ({target_counts} counts)")

    if not send_position_command(ser, TEST_ADDR, target_counts):
        print("  Failed to send command")
        ser.close()
        return

    print("  LOAD_TRAJECTORY sent")

    # Try sending START_MOTION command (0x05)
    print("  Sending START_MOTION command...")
    response = send_command(ser, TEST_ADDR, 0x05, [])
    print(f"  START_MOTION response: {response.hex() if response else 'no response'}")

    # Monitor position for 2 seconds
    print(f"\n=== MONITORING POSITION ===")
    print(f"Time(s)  Position(counts)  Position(mm)   Delta(mm)  Servo  Status")
    print("-" * 70)

    start_time = time.time()

    for i in range(20):  # 2 seconds at 10Hz
        elapsed = time.time() - start_time
        status = read_drive_position(ser, TEST_ADDR)

        if status:
            pos_counts = status['position']
            pos_mm = pos_counts / SCALE
            delta_mm = (pos_counts - initial_pos) / SCALE

            print(f"{elapsed:6.2f}  {pos_counts:16d}  {pos_mm:12.4f}  {delta_mm:10.4f}  {str(status['servo_on']):5s}  0x{status['status']:02X}")
        else:
            print(f"{elapsed:6.2f}  ERROR reading position")

        time.sleep(0.1)

    # Read final position
    print(f"\n=== FINAL POSITION ===")
    status = read_drive_position(ser, TEST_ADDR)
    if status:
        final_pos = status['position']
        moved_mm = (final_pos - initial_pos) / SCALE
        print(f"  Initial: {initial_pos} counts ({initial_pos/SCALE:.4f} mm)")
        print(f"  Final: {final_pos} counts ({final_pos/SCALE:.4f} mm)")
        print(f"  Moved: {final_pos - initial_pos} counts ({moved_mm:.4f} mm)")
        print(f"  Target: {target_counts} counts ({target_mm:.4f} mm)")
        print(f"  Error: {final_pos - target_counts} counts ({(final_pos - target_counts)/SCALE:.4f} mm)")

    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
