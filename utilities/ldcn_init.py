#!/usr/bin/env python3
"""
ldcn_init.py - LDCN Network Initialization Utility

This utility initializes an LDCN network by:
- Auto-detecting and resetting if needed
- Addressing all devices
- Switching to target baud rate
- Configuring I/O controller

Usage:
    python3 ldcn_init.py [options]

Options:
    --port PORT         Serial port (default: /dev/ttyUSB0)
    --num-servos N      Number of servo drives (default: 5)
    --target-baud BAUD  Target baud rate (default: 125000)
    --verbose           Show detailed communication

Example:
    python3 ldcn_init.py --port /dev/ttyUSB0 --target-baud 125000

Author: LinuxCNC Community
License: GPL v2 or later
"""

import serial
import time
import argparse
import sys

class LDCNInitializer:
    """LDCN Network Initializer"""
    
    HEADER = 0xAA
    CMD_NOP = 0x0E
    CMD_HARD_RESET = 0x0F
    CMD_SET_ADDRESS = 0x01
    CMD_SET_BAUD = 0x0A
    CMD_DEFINE_STATUS = 0x02
    
    BRD_VALUES = {
        9600: 0x81,
        19200: 0x3F,
        57600: 0x14,
        115200: 0x0A,
        125000: 0x27,
        312500: 0x0F,
        625000: 0x07,
        1250000: 0x03
    }
    
    COMMON_BAUDS = [19200, 125000, 115200, 57600, 9600]
    
    def __init__(self, port='/dev/ttyUSB0', num_servos=5, target_baud=125000, verbose=False):
        self.port = port
        self.num_servos = num_servos
        self.target_baud = target_baud
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
            print(f"    TX: {packet.hex()}")
        
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.02)
        
        response = self.ser.read(50)
        
        if self.verbose:
            print(f"    RX: {response.hex()}")
        
        return response
    
    def open_serial(self, baud):
        """Open serial port at specified baud rate"""
        if self.ser:
            self.ser.close()
            time.sleep(0.2)
        
        self.ser = serial.Serial(self.port, baud, timeout=0.2)
        time.sleep(0.2)
    
    def auto_detect_baud(self):
        """Auto-detect current baud rate"""
        self.log("Auto-detecting current baud rate...")
        
        for baud in self.COMMON_BAUDS:
            if self.verbose:
                self.log(f"  Trying {baud}...", 'DEBUG')
            
            try:
                self.open_serial(baud)
                
                # Try common addresses
                for addr in [1, 2, 3, 6]:
                    response = self.send_command(addr, self.CMD_NOP)
                    if len(response) >= 2:
                        self.log(f"  ✓ Devices found at {baud} baud")
                        return baud
            except:
                pass
        
        self.log("  Could not detect baud rate, assuming 19200", 'WARN')
        return 19200
    
    def hard_reset(self):
        """Send hard reset command"""
        self.log("  Sending hard reset...")
        
        packet = bytes([self.HEADER, 0xFF, 0x0F, 0x0E])
        self.ser.write(packet)
        self.ser.flush()
        
        self.log("  Waiting 2s for reset...")
        time.sleep(2.0)
        
        # Flush input buffer
        self.ser.reset_input_buffer()
    
    def address_devices(self):
        """Address all devices sequentially"""
        total = self.num_servos + 1  # servos + I/O controller
        self.log(f"  Addressing {total} devices...")
        
        addressed = 0
        for i in range(1, total + 1):
            response = self.send_command(0x00, self.CMD_SET_ADDRESS, [i, 0xFF])
            
            if len(response) >= 2:
                self.log(f"    Device {i}: ✓ Addressed")
                addressed += 1
            else:
                self.log(f"    Device {i}: ⚠ No response", 'WARN')
            
            time.sleep(0.3)
        
        self.log("  Waiting 2s for stabilization...")
        time.sleep(2.0)
        
        return addressed
    
    def verify_devices(self):
        """Verify all devices are responding"""
        total = self.num_servos + 1
        responding = 0
        
        for addr in range(1, total + 1):
            response = self.send_command(addr, self.CMD_NOP)
            if len(response) >= 2:
                responding += 1
        
        return responding
    
    def change_baud_rate(self, new_baud):
        """Change baud rate of all devices"""
        if new_baud not in self.BRD_VALUES:
            self.log(f"  Unsupported baud rate: {new_baud}", 'ERROR')
            return False
        
        brd = self.BRD_VALUES[new_baud]
        self.log(f"  Sending baud rate command (BRD=0x{brd:02X})...")
        
        packet = bytes([self.HEADER, 0xFF, 0x1A, brd, (0xFF + 0x1A + brd) & 0xFF])
        self.ser.write(packet)
        self.ser.flush()
        
        time.sleep(0.5)
        
        # Reopen at new baud rate
        self.log(f"  Reopening at {new_baud} baud...")
        self.open_serial(new_baud)
        
        return True
    
    def configure_io_controller(self):
        """Configure I/O controller for full status"""
        self.log("  Configuring I/O controller...")
        self.send_command(6, self.CMD_DEFINE_STATUS, [0xFF, 0xFF])
        time.sleep(1.0)
    
    def initialize(self):
        """Complete initialization sequence"""
        try:
            self.log(f"\nLDCN Network Initialization")
            self.log(f"Port: {self.port}")
            self.log(f"Servos: {self.num_servos}")
            self.log(f"Target baud: {self.target_baud}\n")
            
            # Step 1: Auto-detect baud rate
            self.log("Step 1: Auto-detect current baud rate")
            current_baud = self.auto_detect_baud()
            
            # Step 2: Reset if not at 19200
            if current_baud != 19200:
                self.log(f"\nStep 2: Reset to 19200 baud (currently {current_baud})")
                self.open_serial(current_baud)
                self.hard_reset()
                current_baud = 19200
            else:
                self.log("\nStep 2: Already at 19200 baud")
            
            # Step 3: Open at 19200 and reset
            self.log("\nStep 3: Hard reset at 19200")
            self.open_serial(19200)
            self.hard_reset()
            
            # Step 4: Address devices
            self.log("\nStep 4: Address devices")
            addressed = self.address_devices()
            total = self.num_servos + 1
            
            if addressed < total:
                self.log(f"  ⚠ Only {addressed}/{total} devices addressed", 'WARN')
            
            # Step 5: Verify at 19200
            self.log("\nStep 5: Verify at 19200 baud")
            responding = self.verify_devices()
            self.log(f"  {responding}/{total} devices responding")
            
            if responding < total:
                self.log("  ⚠ Not all devices responding", 'WARN')
            
            # Step 6: Change baud rate
            if self.target_baud != 19200:
                self.log(f"\nStep 6: Switch to {self.target_baud} baud")
                if not self.change_baud_rate(self.target_baud):
                    return False
                
                # Step 7: Verify at new baud
                self.log(f"\nStep 7: Verify at {self.target_baud} baud")
                responding = self.verify_devices()
                self.log(f"  {responding}/{total} devices responding")
                
                if responding < total:
                    self.log(f"  ✗ Communication failed at {self.target_baud}", 'ERROR')
                    return False
            else:
                self.log("\nStep 6: Target baud is 19200 (no change needed)")
            
            # Step 8: Configure I/O controller
            self.log(f"\nStep 8: Configure I/O controller")
            self.configure_io_controller()
            
            self.log("\n✓ Initialization complete!")
            self.log(f"  Network ready at {self.target_baud} baud")
            return True
            
        except KeyboardInterrupt:
            self.log("\n✗ Interrupted by user", 'ERROR')
            return False
        except Exception as e:
            self.log(f"\n✗ Error: {e}", 'ERROR')
            import traceback
            traceback.print_exc()
            return False
        finally:
            if self.ser:
                self.ser.close()

def main():
    parser = argparse.ArgumentParser(
        description='LDCN Network Initialization Utility',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--num-servos', type=int, default=5,
                       help='Number of servo drives (default: 5)')
    parser.add_argument('--target-baud', type=int, default=125000,
                       help='Target baud rate (default: 125000)')
    parser.add_argument('--verbose', action='store_true',
                       help='Show detailed communication')
    
    args = parser.parse_args()
    
    init = LDCNInitializer(
        port=args.port,
        num_servos=args.num_servos,
        target_baud=args.target_baud,
        verbose=args.verbose
    )
    
    success = init.initialize()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()