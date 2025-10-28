#!/usr/bin/env python3
"""
ldcn_monitor.py - LDCN Network Status Monitor

This utility continuously monitors LDCN network status, displaying:
- Communication status with all devices
- Power-on state from I/O controller
- Real-time status updates

Usage:
    python3 ldcn_monitor.py [options]

Options:
    --port PORT       Serial port (default: /dev/ttyUSB0)
    --baud BAUD       Baud rate (default: auto-detect)
    --interval SEC    Update interval in seconds (default: 1.0)

Example:
    python3 ldcn_monitor.py --port /dev/ttyUSB0 --interval 0.5

Author: LinuxCNC Community
License: GPL v2 or later
"""

import serial
import time
import argparse
import sys
from datetime import datetime

class LDCNMonitor:
    """LDCN Network Status Monitor"""
    
    HEADER = 0xAA
    CMD_NOP = 0x0E
    CMD_DEFINE_STATUS = 0x02
    
    COMMON_BAUDS = [19200, 125000, 115200, 57600, 9600]
    
    def __init__(self, port='/dev/ttyUSB0', baud=None, interval=1.0):
        self.port = port
        self.baud = baud
        self.interval = interval
        self.ser = None
        self.device_names = {
            1: "Servo 1",
            2: "Servo 2",
            3: "Servo 3",
            4: "Servo 4",
            5: "Servo 5",
            6: "I/O Ctrl"
        }
    
    def send_command(self, address, command, data=[]):
        """Send LDCN command and return response"""
        header = self.HEADER
        num_data = len(data)
        cmd_byte = (num_data << 4) | (command & 0x0F)
        checksum = (address + cmd_byte + sum(data)) & 0xFF
        packet = bytes([header, address, cmd_byte] + data + [checksum])
        
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.02)
        
        return self.ser.read(50)
    
    def auto_detect_baud(self):
        """Auto-detect current baud rate"""
        print("Auto-detecting baud rate...")
        
        for baud in self.COMMON_BAUDS:
            try:
                if self.ser:
                    self.ser.close()
                
                self.ser = serial.Serial(self.port, baud, timeout=0.1)
                time.sleep(0.2)
                
                for addr in [1, 2, 3, 6]:
                    response = self.send_command(addr, self.CMD_NOP)
                    if len(response) >= 2:
                        print(f"✓ Detected {baud} baud\n")
                        return baud
            except:
                pass
        
        print("✗ Could not detect baud rate\n")
        return None
    
    def configure_io_controller(self):
        """Configure I/O controller for full status"""
        self.send_command(6, self.CMD_DEFINE_STATUS, [0xFF, 0xFF])
        time.sleep(0.5)
    
    def read_device_status(self, address):
        """Read status from a device"""
        response = self.send_command(address, self.CMD_NOP)
        
        if len(response) >= 2:
            # For I/O controller with full status, status is at index 1
            # For servo drives, status is at index 0
            if address == 6 and len(response) > 2:
                status = response[1]
            else:
                status = response[0]
            
            return {
                'online': True,
                'status': status,
                'power_on': bool(status & 0x08) if address == 6 else None
            }
        else:
            return {
                'online': False,
                'status': None,
                'power_on': None
            }
    
    def format_status_bits(self, status):
        """Format status byte as bit string"""
        return f"{status:08b}"
    
    def print_header(self):
        """Print monitor header"""
        print("\n" + "="*70)
        print("  LDCN Network Monitor")
        print("  " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        print("="*70)
        print(f"{'Device':<12} {'Status':<8} {'Bits':<10} {'Power':<8}")
        print("-"*70)
    
    def print_device_status(self, address, info):
        """Print status for one device"""
        name = self.device_names.get(address, f"Device {address}")
        
        if info['online']:
            status_hex = f"0x{info['status']:02X}"
            bits = self.format_status_bits(info['status'])
            power = "ON" if info['power_on'] else "OFF" if info['power_on'] is not None else "-"
            print(f"{name:<12} {'✓':<8} {status_hex:<10} {power:<8}")
        else:
            print(f"{name:<12} {'✗ OFFLINE':<8} {'-':<10} {'-':<8}")
    
    def monitor(self):
        """Main monitoring loop"""
        try:
            print("LDCN Network Status Monitor")
            print(f"Port: {self.port}")
            print(f"Update interval: {self.interval}s\n")
            
            # Auto-detect or open at specified baud
            if self.baud is None:
                self.baud = self.auto_detect_baud()
                if self.baud is None:
                    return False
            else:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
                time.sleep(0.2)
            
            print(f"Using {self.baud} baud")
            
            # Configure I/O controller
            print("Configuring I/O controller...")
            self.configure_io_controller()
            
            print("\nMonitoring... (Ctrl+C to stop)\n")
            
            iteration = 0
            while True:
                # Print header every 20 iterations
                if iteration % 20 == 0:
                    self.print_header()
                
                # Read all devices
                for addr in range(1, 7):
                    info = self.read_device_status(addr)
                    self.print_device_status(addr, info)
                
                print()
                
                iteration += 1
                time.sleep(self.interval)
            
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")
            return True
        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            if self.ser:
                self.ser.close()

def main():
    parser = argparse.ArgumentParser(
        description='LDCN Network Status Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int,
                       help='Baud rate (default: auto-detect)')
    parser.add_argument('--interval', type=float, default=1.0,
                       help='Update interval in seconds (default: 1.0)')
    
    args = parser.parse_args()
    
    monitor = LDCNMonitor(
        port=args.port,
        baud=args.baud,
        interval=args.interval
    )
    
    success = monitor.monitor()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()