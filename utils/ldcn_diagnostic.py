#!/usr/bin/env python3
"""
ldcn_diagnostic.py - LDCN Network Diagnostic Utility

This utility helps diagnose LDCN network issues by:
- Auto-detecting current baud rate
- Testing communication with all devices
- Displaying device status
- Verifying power control functionality

Usage:
    python3 ldcn_diagnostic.py [options]

Options:
    --port PORT       Serial port (default: /dev/ttyUSB0)
    --verbose         Show detailed communication
    --test-power      Test power-on detection

Example:
    python3 ldcn_diagnostic.py --port /dev/ttyUSB0 --verbose

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
    
    def test_device(self, address, name=""):
        """Test communication with a single device"""
        response = self.send_command(address, self.CMD_NOP)
        
        if len(response) >= 2:
            status = response[0] if len(response) == 2 else response[1]
            self.log(f"  Device {address} ({name}): ✓ OK (status: 0x{status:02X})")
            return True
        else:
            self.log(f"  Device {address} ({name}): ✗ NO RESPONSE", 'WARN')
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
    
    def run(self, test_power=False):
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
    
    args = parser.parse_args()
    
    diag = LDCNDiagnostic(port=args.port, verbose=args.verbose)
    success = diag.run(test_power=args.test_power)
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
