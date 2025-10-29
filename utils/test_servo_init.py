#!/usr/bin/env python3
"""
Test servo initialization sequence - mimics what the C code does during F2 power-up
to diagnose why drives fault after initialization.
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

    print(f"  TX to addr {address}: {packet.hex()}")
    ser.write(packet)
    ser.flush()
    time.sleep(0.02)

    response = ser.read(50)
    if len(response) > 0:
        print(f"  RX: {response.hex()}")
    return response

def parse_servo_status(response, status_bits):
    """Parse servo drive status response"""
    if len(response) < 2:
        return None

    status_byte = response[0]
    data = response[1:-1]  # Everything except first byte and checksum

    result = {
        'status_byte': status_byte,
        'move_done': bool(status_byte & 0x01),
        'checksum_error': bool(status_byte & 0x02),
        'current_limit': bool(status_byte & 0x04),
        'power_on': bool(status_byte & 0x08),
        'position_error': bool(status_byte & 0x10),
        'home_source': bool(status_byte & 0x20),
        'limit2': bool(status_byte & 0x40),
        'home_in_progress': bool(status_byte & 0x80),
    }

    idx = 0

    # Position (4 bytes)
    if status_bits & 0x01 and len(data) >= idx + 4:
        result['position'] = struct.unpack('<i', bytes(data[idx:idx+4]))[0]
        idx += 4

    # A/D value (1 byte)
    if status_bits & 0x02 and len(data) >= idx + 1:
        result['ad_value'] = data[idx]
        idx += 1

    # Velocity (2 bytes)
    if status_bits & 0x04 and len(data) >= idx + 2:
        result['velocity'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
        idx += 2

    # Auxiliary status (1 byte)
    if status_bits & 0x08 and len(data) >= idx + 1:
        aux = data[idx]
        result['servo_on'] = bool(aux & 0x04)
        result['servo_overrun'] = bool(aux & 0x20)
        idx += 1

    # Position error (2 bytes)
    if status_bits & 0x40 and len(data) >= idx + 2:
        result['following_error'] = struct.unpack('<h', bytes(data[idx:idx+2]))[0]
        idx += 2

    return result

def initialize_servo(ser, addr):
    """Initialize a single servo drive - mimics init_drive() from C code"""
    print(f"\n=== INITIALIZING SERVO {addr} ===")

    # Step 1: Define status reporting
    print(f"Step 1: Define status (pos, vel, aux, pos_err)")
    status_bits = 0x01 | 0x04 | 0x08 | 0x40  # pos, vel, aux, pos_err
    send_command(ser, addr, 0x02, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])
    time.sleep(0.1)

    # Step 2: Set PID gains (conservative values)
    print(f"Step 2: Set PID gains")
    kp = 2
    kd = 50
    ki = 0
    il = 40
    ol = 255
    cl = 0
    el = 2000
    sr = 20
    db = 0

    gain_data = struct.pack('<HHHBBBHB', kp, kd, ki, il, ol, cl, el, sr)
    send_command(ser, addr, 0x06, list(gain_data) + [db])
    time.sleep(0.1)

    # Step 3: Load initial trajectory
    print(f"Step 3: Load trajectory")
    traj_ctrl = 0x10  # servo_mode=1, others=0
    traj_data = [traj_ctrl, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0]  # pos=0, vel=0, accel=100
    send_command(ser, addr, 0x04, traj_data)
    time.sleep(0.1)

    # Step 4: Enable servo (STOP_AMP_ENABLE | STOP_ABRUPT)
    print(f"Step 4: Enable servo")
    stop_ctrl = 0x01 | 0x04  # AMP_ENABLE | ABRUPT
    send_command(ser, addr, 0x07, [stop_ctrl])
    time.sleep(0.1)

    # Step 5: Reset position counter
    print(f"Step 5: Reset position")
    send_command(ser, addr, 0x00, [])
    time.sleep(0.1)

    # Step 6: Clear sticky status bits
    print(f"Step 6: Clear sticky bits")
    send_command(ser, addr, 0x0B, [])
    time.sleep(0.1)

    # Step 7: Read status
    print(f"Step 7: Read status")
    response = send_command(ser, addr, 0x03, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])

    status = parse_servo_status(response, status_bits)
    if status:
        print(f"\n  Servo {addr} Status:")
        print(f"    Status byte: 0x{status['status_byte']:02X}")
        print(f"    Power on: {status.get('power_on', 'N/A')}")
        print(f"    Servo on: {status.get('servo_on', 'N/A')}")
        print(f"    Current limit: {status.get('current_limit', 'N/A')}")
        print(f"    Position error: {status.get('position_error', 'N/A')}")
        print(f"    Position: {status.get('position', 'N/A')}")
        print(f"    Velocity: {status.get('velocity', 'N/A')}")
        print(f"    Following error: {status.get('following_error', 'N/A')}")

        # Highlight faults
        if status.get('current_limit'):
            print(f"    ⚠️  CURRENT_LIMIT FLAG SET!")
        if status.get('position_error'):
            print(f"    ⚠️  POSITION_ERROR FLAG SET!")

    return status

def try_communicate_at_baud(port, baud):
    """Try to communicate at a specific baud rate"""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.2)
        for addr in [1, 2, 3, 6]:
            header = 0xAA
            cmd_byte = 0x0E  # NOP
            checksum = (addr + cmd_byte) & 0xFF
            packet = bytes([header, addr, cmd_byte, checksum])
            ser.write(packet)
            ser.flush()
            time.sleep(0.02)
            response = ser.read(10)
            if len(response) >= 2:
                print(f"    ✓ Device {addr} responded at {baud} baud")
                ser.close()
                return True
        ser.close()
        return False
    except:
        return False

def find_current_baud(port):
    """Find which baud rate the devices are currently using"""
    print("=== AUTO-DETECTING CURRENT BAUD RATE ===")
    baud_rates = [19200, 125000, 115200, 57600, 9600]

    for baud in baud_rates:
        print(f"  Trying {baud} baud...")
        if try_communicate_at_baud(port, baud):
            print(f"  ✓ Found devices at {baud} baud!")
            return baud

    print("  ✗ No devices found, assuming 19200")
    return 19200

def main():
    PORT = '/dev/ttyUSB0'
    BAUD_TARGET = 125000

    print("="*70)
    print("COMPLETE SERVO INITIALIZATION TEST")
    print("Full sequence: reset → address → baud upgrade → servo init")
    print("="*70)

    # Auto-detect current baud
    current_baud = find_current_baud(PORT)

    # If not at 19200, reset
    if current_baud != 19200:
        print(f"\n=== RESETTING FROM {current_baud} TO 19200 ===")
        ser = serial.Serial(PORT, current_baud, timeout=0.1)
        time.sleep(0.2)
        reset_packet = bytes([0xAA, 0xFF, 0x0F, 0x0E])
        ser.write(reset_packet)
        ser.flush()
        time.sleep(2.0)
        ser.close()
        time.sleep(0.5)
        current_baud = 19200

    # Open at 19200
    print(f"\n=== OPENING AT {current_baud} BAUD ===")
    ser = serial.Serial(PORT, current_baud, timeout=0.2)
    time.sleep(0.2)

    # Hard reset
    print("\n=== HARD RESET ===")
    reset_packet = bytes([0xAA, 0xFF, 0x0F, 0x0E])
    ser.write(reset_packet)
    ser.flush()
    time.sleep(2.0)
    ser.reset_input_buffer()

    # Address devices
    print("\n=== ADDRESSING 6 DEVICES ===")
    for i in range(1, 7):
        response = send_command(ser, 0x00, 0x01, [i, 0xFF])
        status = "✓" if len(response) >= 2 else "✗"
        print(f"  Device {i}: {status}")
        time.sleep(0.3)
    time.sleep(2.0)

    # Upgrade to target baud
    print(f"\n=== UPGRADING TO {BAUD_TARGET} BAUD ===")
    brd = 0x27  # 125000 baud
    packet = bytes([0xAA, 0xFF, 0x1A, brd, (0xFF + 0x1A + brd) & 0xFF])
    ser.write(packet)
    ser.flush()
    time.sleep(0.5)
    ser.close()
    time.sleep(0.5)

    # Reopen at new baud
    ser = serial.Serial(PORT, BAUD_TARGET, timeout=0.2)
    time.sleep(0.5)

    # Configure supervisor
    print("\n=== CONFIGURING SUPERVISOR ===")
    send_command(ser, 6, 0x02, [0xFF, 0xFF])
    time.sleep(1.0)
    ser.reset_input_buffer()

    # Read supervisor baseline - power should be OFF after reset
    print("\n=== SUPERVISOR BASELINE (power should be OFF) ===")
    baseline_status = 0x00
    for attempt in range(5):
        response = send_command(ser, 6, 0x0E, [])
        if len(response) > 1:
            baseline_status = response[1]
            power = bool(baseline_status & 0x08)
            print(f"  Attempt {attempt}: Status: 0x{baseline_status:02X}, Power: {power}")
            if baseline_status != 0x00:
                break
        time.sleep(0.2)

    print("\n" + "="*70)
    print("*** POWER BUTTON LED SHOULD BE FLASHING GREEN NOW ***")
    print("*** PRESS THE PHYSICAL POWER BUTTON ***")
    print("="*70)
    print("\nWaiting for power button press (polling supervisor)...")

    # Poll for power button press (like wait_for_power() does)
    power_detected = False
    for i in range(50):  # 10 seconds timeout
        response = send_command(ser, 6, 0x0E, [])
        if len(response) > 1:
            status = response[1]
            power = bool(status & 0x08)

            # Print every 5th poll or when changed
            if i % 5 == 0 or status != baseline_status:
                print(f"  [{i:2d}] Status: 0x{status:02X}, Power: {power}")

            if status != baseline_status and power:
                print(f"\n✓ Power button pressed! Status changed: 0x{baseline_status:02X} → 0x{status:02X}")
                power_detected = True
                break

            baseline_status = status
        time.sleep(0.2)

    if not power_detected:
        print("\n✗ Power button not detected (timeout)")
        ser.close()
        return

    # Initialize all 5 servo drives
    print("\n" + "="*70)
    print("INITIALIZING 5 SERVO DRIVES")
    print("="*70)

    start_time = time.time()
    drive_status = {}

    for addr in range(1, 6):
        drive_status[addr] = initialize_servo(ser, addr)

    elapsed = time.time() - start_time
    print(f"\n✓ All servos initialized in {elapsed:.3f} seconds")

    # Poll supervisor immediately after init
    print("\n=== SUPERVISOR STATUS AFTER INIT ===")
    for i in range(5):
        response = send_command(ser, 6, 0x0E, [])
        if len(response) > 1:
            status = response[1]
            power = bool(status & 0x08)
            current_limit = bool(status & 0x04)
            print(f"  [{i}] Status: 0x{status:02X}, Power: {power}, Current limit: {current_limit}")
        time.sleep(0.1)

    # Read drive status again after a short delay
    print("\n=== DRIVE STATUS 1 SECOND AFTER INIT ===")
    time.sleep(1.0)

    status_bits = 0x01 | 0x04 | 0x08 | 0x40
    for addr in range(1, 6):
        print(f"\nDrive {addr}:")
        response = send_command(ser, addr, 0x03, [status_bits & 0xFF, (status_bits >> 8) & 0xFF])
        status = parse_servo_status(response, status_bits)
        if status:
            print(f"  Status byte: 0x{status['status_byte']:02X}")
            print(f"  Servo on: {status.get('servo_on', 'N/A')}")
            print(f"  Current limit: {status.get('current_limit', 'N/A')}")
            print(f"  Position error: {status.get('position_error', 'N/A')}")
            if status.get('current_limit'):
                print(f"  ⚠️  CURRENT_LIMIT STILL SET!")
            if status.get('position_error'):
                print(f"  ⚠️  POSITION_ERROR STILL SET!")

    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
