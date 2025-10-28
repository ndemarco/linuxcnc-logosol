#!/usr/bin/env python3
"""
ldcn_diagnostic.py - LDCN Network Diagnostic Utility

This utility helps diagnose LDCN network issues by:
- Auto-detecting current baud rate
- Testing communication with all devices
- Displaying device status and fault conditions
- Verifying power control functionality
- Monitoring servo faults (checksum errors, current limit, position error)

Usage:
    python3 ldcn_diagnostic.py [options]

Options:
    --port PORT       Serial port (default: /dev/ttyUSB0)
    --verbose         Show detailed communication
    --test-power      Test power-on detection
    --test-faults     Check all servos for fault conditions

Example:
    python3 ldcn_diagnostic.py --port /dev/ttyUSB0 --verbose --test-faults

Author: LinuxCNC Community
License: GPL v2 or later
"""

import serial
import time
import argparse
import sys

class LDCNDiagnostic:
    """LDCN Network Diagnostic Tool"""

    HEADER = 0xAA
    CMD_NOP = 0x0E
    CMD_HARD_RESET = 0x0F
    CMD_SET_ADDRESS = 0x01
    CMD_DEFINE_STATUS = 0x02

    # Status bit flags
    STATUS_MOVE_DONE = (1 << 0)
    STATUS_CKSUM_ERROR = (1 << 1)
    STATUS_CURRENT_LIMIT = (1 << 2)
    STATUS_POWER_ON = (1 << 3)
    STATUS_POS_ERROR = (1 << 4)
    STATUS_HOME_SOURCE = (1 << 5)
    STATUS_LIMIT2 = (1 << 6)
    STATUS_HOME_IN_PROG = (1 << 7)

    COMMON_BAUDS = [19200, 125000, 115200, 57600, 9600, 38400]
    
    def __init__(self, port='/dev/ttyUSB0', verbose=False):
        self.port = port
        self.verbose = verbose
        self.ser = None
    
    def log(self, msg, level='INFO'):
        """Print log message"""
        print(f"[{level}] {msg}")
    
    def send_command(self, address, command, data=[]):
        """Send LDCN command and return response"""
        header = self.HEADER
        num_data = len(data)
        cmd_byte = (num_data << 4) | (command & 0x0F)
        checksum = (address + cmd_byte + sum(data)) & 0xFF
        packet = bytes([header, address, cmd_byte] + data + [checksum])
        
        if self.verbose:
            print(f"  TX: {packet.hex()}")
        
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.02)
        
        response = self.ser.read(50)
        
        if self.verbose:
            print(f"  RX: {response.hex()} ({len(response)} bytes)")
        
        return response
    
    def try_baud(self, baud):
        """Try to communicate at a specific baud rate"""
        try:
            if self.ser:
                self.ser.close()
            
            self.ser = serial.Serial(self.port, baud, timeout=0.1)
            time.sleep(0.2)
            
            # Try to communicate with common addresses
            for addr in [1, 2, 3, 6]:
                response = self.send_command(addr, self.CMD_NOP)
                if len(response) >= 2:
                    return True
            
            return False
        except Exception as e:
            if self.verbose:
                self.log(f"Error at {baud} baud: {e}", 'WARN')
            return False
    
    def auto_detect_baud(self):
        """Auto-detect current baud rate"""
        self.log("Auto-detecting baud rate...")
        
        for baud in self.COMMON_BAUDS:
            self.log(f"  Trying {baud} baud...")
            if self.try_baud(baud):
                self.log(f"  ✓ Devices found at {baud} baud!", 'INFO')
                return baud
        
        self.log("  ✗ No devices found at any common baud rate", 'ERROR')
        return None
    
    def check_faults(self, status):
        """Check status byte for fault conditions"""
        faults = []

        if status & self.STATUS_CKSUM_ERROR:
            faults.append("CHECKSUM_ERROR")

        if status & self.STATUS_CURRENT_LIMIT:
            faults.append("CURRENT_LIMIT")

        if status & self.STATUS_POS_ERROR:
            faults.append("POSITION_ERROR")

        return faults

    def decode_status(self, status):
        """Decode status byte into human-readable flags"""
        flags = []

        if status & self.STATUS_MOVE_DONE:
            flags.append("MOVE_DONE")
        if status & self.STATUS_CKSUM_ERROR:
            flags.append("CKSUM_ERR")
        if status & self.STATUS_CURRENT_LIMIT:
            flags.append("CURRENT_LIM")
        if status & self.STATUS_POWER_ON:
            flags.append("POWER_ON")
        if status & self.STATUS_POS_ERROR:
            flags.append("POS_ERR")
        if status & self.STATUS_HOME_SOURCE:
            flags.append("HOME_SRC")
        if status & self.STATUS_LIMIT2:
            flags.append("LIMIT2")
        if status & self.STATUS_HOME_IN_PROG:
            flags.append("HOMING")

        return flags if flags else ["NONE"]

    def test_device(self, address, name=""):
        """Test communication with a single device"""
        response = self.send_command(address, self.CMD_NOP)

        if len(response) >= 2:
            status = response[0] if len(response) == 2 else response[1]
            faults = self.check_faults(status)

            if faults:
                fault_str = ", ".join(faults)
                self.log(f"  Device {address} ({name}): ⚠ FAULT (0x{status:02X}) - {fault_str}", 'WARN')
            else:
                flags = self.decode_status(status)
                flag_str = ", ".join(flags)
                self.log(f"  Device {address} ({name}): ✓ OK (0x{status:02X}) [{flag_str}]")

            return True
        else:
            self.log(f"  Device {address} ({name}): ✗ NO RESPONSE", 'ERROR')
            return False
    
    def test_all_devices(self):
        """Test communication with all expected devices"""
        self.log("\nTesting all devices...")
        
        devices = [
            (1, "Servo 1"),
            (2, "Servo 2"),
            (3, "Servo 3"),
            (4, "Servo 4"),
            (5, "Servo 5"),
            (6, "I/O Controller")
        ]
        
        results = []
        for addr, name in devices:
            results.append(self.test_device(addr, name))
        
        responding = sum(results)
        self.log(f"\n{responding}/{len(devices)} devices responding")
        
        return responding == len(devices)
    
    def test_faults(self):
        """Continuously monitor servos for fault conditions"""
        self.log("\nContinuous Fault Monitoring")
        self.log("=" * 70)
        self.log("Monitoring servos 1-5 for fault conditions...")
        self.log("Press Ctrl+C to stop\n")

        servos = [(1, "Servo 1"), (2, "Servo 2"), (3, "Servo 3"),
                  (4, "Servo 4"), (5, "Servo 5")]

        total_faults = 0
        iteration = 0

        try:
            while True:
                iteration += 1
                any_faults = False

                for addr, name in servos:
                    response = self.send_command(addr, self.CMD_NOP)

                    if len(response) >= 2:
                        status = response[0] if len(response) == 2 else response[1]
                        faults = self.check_faults(status)

                        if faults:
                            fault_str = ", ".join(faults)
                            self.log(f"[{iteration:04d}] {name}: ⚠ FAULT - {fault_str}", 'WARN')
                            any_faults = True
                            total_faults += 1

                if iteration % 10 == 0 and not any_faults:
                    self.log(f"[{iteration:04d}] All servos OK - No faults detected")

                time.sleep(0.2)

        except KeyboardInterrupt:
            self.log(f"\n\nMonitoring stopped after {iteration} iterations")
            self.log(f"Total faults detected: {total_faults}")

        return total_faults == 0

    def test_power_detection(self):
        """Test power-on detection from I/O controller"""
        self.log("\nTesting power-on detection...")
        self.log("Configuring I/O controller for full status...")

        # Configure device 6 for full status
        self.send_command(6, self.CMD_DEFINE_STATUS, [0xFF, 0xFF])
        time.sleep(1.0)

        self.log("\nBaseline (power should be OFF):")
        for i in range(3):
            response = self.send_command(6, self.CMD_NOP)
            if len(response) > 1:
                status = response[1]
                power = bool(status & 0x08)
                self.log(f"  Status: 0x{status:02X}, Power: {power}")
            time.sleep(0.2)

        self.log("\n" + "="*70)
        self.log("*** PRESS THE POWER BUTTON NOW ***")
        self.log("="*70)
        input("Press ENTER after pressing power button...")

        self.log("\nMonitoring for power-on...")
        last_status = 0

        for i in range(30):
            response = self.send_command(6, self.CMD_NOP)

            if len(response) > 1:
                status = response[1]
                power = bool(status & 0x08)

                if status != last_status or i % 5 == 0:
                    self.log(f"  [{i:2d}] Status: 0x{status:02X}, Power: {power}")

                if power and not (last_status & 0x08):
                    self.log("\n!!! POWER DETECTED !!!", 'INFO')
                    return True

                last_status = status

            time.sleep(0.5)

        self.log("\n✗ Power not detected within timeout", 'ERROR')
        return False
    
    def run(self, test_power=False, test_faults=False):
        """Run diagnostic tests"""
        try:
            self.log(f"LDCN Network Diagnostic Tool")
            self.log(f"Port: {self.port}\n")

            # Auto-detect baud rate
            current_baud = self.auto_detect_baud()
            if not current_baud:
                self.log("Cannot proceed without communication", 'ERROR')
                return False

            # Test all devices
            all_ok = self.test_all_devices()

            # Test power detection if requested
            if test_power and all_ok:
                self.test_power_detection()

            # Test faults if requested
            if test_faults and all_ok:
                self.test_faults()

            self.log("\nDiagnostic complete!")
            return all_ok

        except KeyboardInterrupt:
            self.log("\nInterrupted by user", 'WARN')
            return False
        except Exception as e:
            self.log(f"Error: {e}", 'ERROR')
            import traceback
            traceback.print_exc()
            return False
        finally:
            if self.ser:
                self.ser.close()

def main():
    parser = argparse.ArgumentParser(
        description='LDCN Network Diagnostic Utility',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed communication')
    parser.add_argument('--test-power', action='store_true',
                       help='Test power-on detection')
    parser.add_argument('--test-faults', action='store_true',
                       help='Continuously monitor servos for faults')

    args = parser.parse_args()

    diag = LDCNDiagnostic(port=args.port, verbose=args.verbose)
    success = diag.run(test_power=args.test_power, test_faults=args.test_faults)

    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
