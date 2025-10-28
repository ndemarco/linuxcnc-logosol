#!/usr/bin/env python3
"""
Complete LDCN Network Initialization with Auto-detect and Power Monitoring
"""
import serial
import time

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

def try_communicate_at_baud(port, baud):
    """Try to communicate at a specific baud rate"""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.2)
        for addr in [1, 2, 3, 6]:
            response = send_command(ser, addr, 0x0E, [])
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
    baud_rates = [19200, 125000, 115200, 57600, 9600, 38400]
    
    for baud in baud_rates:
        print(f"  Trying {baud} baud...")
        if try_communicate_at_baud(port, baud):
            print(f"  ✓ Found devices at {baud} baud!")
            return baud
    
    print("  ✗ No devices found at any common baud rate")
    return None

def hard_reset_at_baud(ser, baud):
    """Send hard reset at a specific baud rate"""
    reset_packet = bytes([0xAA, 0xFF, 0x0F, 0x0E])
    print(f"  Sending reset at {baud} baud...")
    ser.write(reset_packet)
    ser.flush()
    time.sleep(2.0)

def main():
    PORT = '/dev/ttyUSB0'
    BAUD_TARGET = 125000
    
    # Auto-detect current baud rate
    current_baud = find_current_baud(PORT)
    
    if current_baud:
        print(f"\n✓ Devices currently at {current_baud} baud")
        
        # If not at 19200, reset to get back to default
        if current_baud != 19200:
            print(f"\nResetting from {current_baud} to 19200...")
            ser = serial.Serial(PORT, current_baud, timeout=0.1)
            time.sleep(0.2)
            hard_reset_at_baud(ser, current_baud)
            ser.close()
            time.sleep(1.0)
            current_baud = 19200
    else:
        print("\n⚠ Could not find devices, assuming 19200")
        current_baud = 19200
    
    # Open at 19200
    print(f"\n=== OPENING AT {current_baud} BAUD ===")
    ser = serial.Serial(PORT, current_baud, timeout=0.2)
    time.sleep(0.2)
    
    # Hard reset to ensure clean state
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
    
    # Verify at 19200
    print("\n=== VERIFYING AT 19200 ===")
    all_ok = True
    for addr in range(1, 7):
        response = send_command(ser, addr, 0x0E, [])
        ok = len(response) >= 2
        status = "✓" if ok else "✗"
        print(f"  Device {addr}: {status}")
        if not ok:
            all_ok = False
    
    if not all_ok:
        print("\n✗ Not all devices responding at 19200")
        ser.close()
        return
    
    # Switch to target baud rate
    print(f"\n=== SWITCHING TO {BAUD_TARGET} BAUD ===")
    brd = 0x27  # 125000 baud
    packet = bytes([0xAA, 0xFF, 0x1A, brd, (0xFF + 0x1A + brd) & 0xFF])
    print(f"  TX: {packet.hex()}")
    ser.write(packet)
    ser.flush()
    time.sleep(0.5)
    
    # Reopen at new baud
    print(f"  Reopening at {BAUD_TARGET} baud...")
    ser.close()
    time.sleep(0.5)
    ser = serial.Serial(PORT, BAUD_TARGET, timeout=0.2)
    time.sleep(0.5)
    
    # Verify at new baud
    print(f"\n=== VERIFYING AT {BAUD_TARGET} ===")
    all_ok = True
    for addr in range(1, 7):
        response = send_command(ser, addr, 0x0E, [])
        ok = len(response) >= 2
        status = "✓" if ok else "✗"
        print(f"  Device {addr}: {status}")
        if not ok:
            all_ok = False
    
    if not all_ok:
        print(f"\n✗ Communication failed at {BAUD_TARGET}")
        ser.close()
        return
    
    print(f"\n✓ All devices at {BAUD_TARGET} baud!")
    
    # Configure 2310g2 for full status
    print("\n=== CONFIGURING 2310g2 ===")
    send_command(ser, 6, 0x02, [0xFF, 0xFF])
    time.sleep(1.0)
    ser.reset_input_buffer()
    
    # Get baseline
    print("\n=== BASELINE (before power on) ===")
    last_status = 0x00
    for attempt in range(5):
        response = send_command(ser, 6, 0x0E, [])
        if len(response) > 1:
            status = response[1]
            power = bool(status & 0x08)
            print(f"  Status: 0x{status:02X}, Power: {power}")
            if status != 0x00:
                last_status = status
                break
        time.sleep(0.2)
    
    print("\n" + "="*70)
    print("*** PRESS POWER BUTTON, THEN PRESS ENTER ***")
    print("="*70)
    input()
    
    # Monitor for power on
    print("\n=== MONITORING FOR POWER ON ===")
    power_detected = False
    
    for i in range(30):
        response = send_command(ser, 6, 0x0E, [])
        
        if len(response) > 1:
            status = response[1]
            power = bool(status & 0x08)
            
            # Print every 5th or when changed
            if i % 5 == 0 or status != last_status:
                print(f"  [{i:2d}] Status: 0x{status:02X}, Power: {power}")
            
            if status != last_status and power:
                print("\n!!! POWER IS NOW ON !!!")
                power_detected = True
                break
            
            last_status = status
        
        time.sleep(0.5)
    
    if power_detected:
        print(f"\n✓ Power detected successfully at {BAUD_TARGET} baud!")
    else:
        print("\n✗ Power not detected")
    
    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()
