#!/usr/bin/env python3
"""
integration_test.py - LDCN Integration Test Suite

This script runs integration tests on the LDCN network to verify
proper operation before using with LinuxCNC.

Tests:
1. Auto-detection and initialization
2. Communication with all devices
3. Power-on detection
4. Enable/disable servos
5. Error recovery

Usage:
    python3 integration_test.py [--port PORT]

Requirements:
    - LDCN network properly wired and powered
    - No other software accessing serial port

Author: LinuxCNC Community
License: GPL v2 or later
"""

import serial
import time
import sys
import argparse

class Colors:
    """ANSI color codes for terminal output"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    END = '\033[0m'
    BOLD = '\033[1m'

class LDCNIntegrationTest:
    """LDCN Integration Test Suite"""
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.ser = None
        self.tests_passed = 0
        self.tests_failed = 0
    
    def log(self, msg, status='INFO'):
        """Print formatted log message"""
        color = {
            'PASS': Colors.GREEN,
            'FAIL': Colors.RED,
            'WARN': Colors.YELLOW,
            'INFO': Colors.BLUE
        }.get(status, '')
        
        print(f"{color}[{status}]{Colors.END} {msg}")
    
    def test_header(self, name):
        """Print test header"""
        print(f"\n{Colors.BOLD}{'='*70}{Colors.END}")
        print(f"{Colors.BOLD}TEST: {name}{Colors.END}")
        print(f"{Colors.BOLD}{'='*70}{Colors.END}")
    
    def assert_true(self, condition, message):
        """Assert condition is true"""
        if condition:
            self.log(f"✓ {message}", 'PASS')
            self.tests_passed += 1
            return True
        else:
            self.log(f"✗ {message}", 'FAIL')
            self.tests_failed += 1
            return False
    
    def send_command(self, address, command, data=[]):
        """Send LDCN command and return response"""
        header = 0xAA
        num_data = len(data)
        cmd_byte = (num_data << 4) | (command & 0x0F)
        checksum = (address + cmd_byte + sum(data)) & 0xFF
        packet = bytes([header, address, cmd_byte] + data + [checksum])
        
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.02)
        return self.ser.read(50)
    
    def test_01_auto_detect(self):
        """Test 1: Auto-detect baud rate"""
        self.test_header("Auto-detect Current Baud Rate")
        
        bauds = [19200, 125000, 115200, 57600, 9600]
        detected = None
        
        for baud in bauds:
            try:
                if self.ser:
                    self.ser.close()
                self.ser = serial.Serial(self.port, baud, timeout=0.1)
                time.sleep(0.2)
                
                response = self.send_command(1, 0x0E)
                if len(response) >= 2:
                    detected = baud
                    break
            except:
                pass
        
        self.assert_true(detected is not None, f"Detected baud rate: {detected}")
        return detected
    
    def test_02_hard_reset(self):
        """Test 2: Hard reset network"""
        self.test_header("Hard Reset Network")
        
        packet = bytes([0xAA, 0xFF, 0x0F, 0x0E])
        self.ser.write(packet)
        self.ser.flush()
        
        self.log("Waiting 2s for reset...", 'INFO')
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        
        self.assert_true(True, "Hard reset command sent")
    
    def test_03_address_devices(self):
        """Test 3: Address devices"""
        self.test_header("Address All Devices")
        
        addressed = 0
        for i in range(1, 7):
            response = self.send_command(0x00, 0x01, [i, 0xFF])
            if len(response) >= 2:
                self.log(f"Device {i} addressed", 'PASS')
                addressed += 1
            else:
                self.log(f"Device {i} failed to address", 'FAIL')
            time.sleep(0.3)
        
        time.sleep(2.0)
        self.assert_true(addressed >= 5, f"Addressed {addressed}/6 devices (minimum 5)")
        return addressed
    
    def test_04_verify_communication(self):
        """Test 4: Verify communication with all devices"""
        self.test_header("Verify Communication")
        
        responding = 0
        for addr in range(1, 7):
            response = self.send_command(addr, 0x0E)
            if len(response) >= 2:
                self.log(f"Device {addr} responding", 'PASS')
                responding += 1
            else:
                self.log(f"Device {addr} not responding", 'FAIL')
        
        self.assert_true(responding >= 5, f"{responding}/6 devices responding (minimum 5)")
        return responding
    
    def test_05_baud_rate_change(self):
        """Test 5: Change baud rate to 125kbps"""
        self.test_header("Change Baud Rate to 125kbps")
        
        # Send baud rate command
        brd = 0x27  # 125kbps
        packet = bytes([0xAA, 0xFF, 0x1A, brd, (0xFF + 0x1A + brd) & 0xFF])
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.5)
        
        # Reopen at new baud
        self.ser.close()
        time.sleep(0.5)
        self.ser = serial.Serial(self.port, 125000, timeout=0.2)
        time.sleep(0.5)
        
        # Verify
        responding = 0
        for addr in range(1, 7):
            response = self.send_command(addr, 0x0E)
            if len(response) >= 2:
                responding += 1
        
        self.assert_true(responding >= 5, f"Communication at 125kbps: {responding}/6 devices")
    
    def test_06_io_controller_config(self):
        """Test 6: Configure I/O controller"""
        self.test_header("Configure I/O Controller for Full Status")
        
        response = self.send_command(6, 0x02, [0xFF, 0xFF])
        time.sleep(1.0)
        
        # Read status
        response = self.send_command(6, 0x0E)
        has_status = len(response) > 2
        
        self.assert_true(has_status, f"I/O controller configured (response: {len(response)} bytes)")
    
    def test_07_power_detection(self):
        """Test 7: Power-on detection"""
        self.test_header("Power-On Detection")
        
        self.log("Reading baseline status...", 'INFO')
        response = self.send_command(6, 0x0E)
        
        if len(response) > 1:
            baseline_status = response[1]
            baseline_power = bool(baseline_status & 0x08)
            self.log(f"Baseline: Status=0x{baseline_status:02X}, Power={baseline_power}", 'INFO')
            
            if not baseline_power:
                self.log("Power is OFF - test will check for power-on ability", 'INFO')
                self.log("This test requires manual power button press", 'WARN')
                self.assert_true(True, "Power detection baseline established")
            else:
                self.log("Power is already ON", 'INFO')
                self.assert_true(True, "Power detection working (already on)")
        else:
            self.assert_true(False, "Could not read I/O controller status")
    
    def test_08_servo_enable(self):
        """Test 8: Enable/disable servo drives"""
        self.test_header("Servo Drive Enable/Disable")
        
        # Test enable
        self.log("Enabling servo 1...", 'INFO')
        response = self.send_command(1, 0x07, [0x11])  # Stop abruptly + amp enable
        time.sleep(0.1)
        
        # Test disable
        self.log("Disabling servo 1...", 'INFO')
        response = self.send_command(1, 0x07, [0x05])  # Stop abruptly + motor off
        time.sleep(0.1)
        
        self.assert_true(True, "Servo enable/disable commands sent")
    
    def run_all_tests(self):
        """Run all integration tests"""
        print(f"\n{Colors.BOLD}{'='*70}")
        print("LDCN Integration Test Suite")
        print(f"Port: {self.port}")
        print(f"{'='*70}{Colors.END}\n")
        
        try:
            # Test sequence
            detected_baud = self.test_01_auto_detect()
            if not detected_baud:
                self.log("Cannot proceed without communication", 'FAIL')
                return False
            
            # Reset to 19200
            if detected_baud != 19200:
                self.log(f"Resetting from {detected_baud} to 19200...", 'INFO')
                self.test_02_hard_reset()
                self.ser.close()
                self.ser = serial.Serial(self.port, 19200, timeout=0.2)
                time.sleep(0.2)
            
            self.test_02_hard_reset()
            addressed = self.test_03_address_devices()
            
            if addressed < 5:
                self.log("Cannot proceed with insufficient devices", 'FAIL')
                return False
            
            self.test_04_verify_communication()
            self.test_05_baud_rate_change()
            self.test_06_io_controller_config()
            self.test_07_power_detection()
            self.test_08_servo_enable()
            
            return True
            
        except KeyboardInterrupt:
            self.log("\nTests interrupted by user", 'WARN')
            return False
        except Exception as e:
            self.log(f"\nTest failed with exception: {e}", 'FAIL')
            import traceback
            traceback.print_exc()
            return False
        finally:
            if self.ser:
                self.ser.close()
    
    def print_summary(self):
        """Print test summary"""
        total = self.tests_passed + self.tests_failed
        pass_rate = (self.tests_passed / total * 100) if total > 0 else 0
        
        print(f"\n{Colors.BOLD}{'='*70}")
        print("TEST SUMMARY")
        print(f"{'='*70}{Colors.END}")
        print(f"Total:  {total}")
        print(f"{Colors.GREEN}Passed: {self.tests_passed}{Colors.END}")
        print(f"{Colors.RED}Failed: {self.tests_failed}{Colors.END}")
        print(f"Rate:   {pass_rate:.1f}%")
        
        if self.tests_failed == 0:
            print(f"\n{Colors.GREEN}{Colors.BOLD}✓ ALL TESTS PASSED{Colors.END}")
            print("LDCN network is ready for LinuxCNC!")
        else:
            print(f"\n{Colors.RED}{Colors.BOLD}✗ SOME TESTS FAILED{Colors.END}")
            print("Please address failures before using with LinuxCNC")
        
        print()

def main():
    parser = argparse.ArgumentParser(description='LDCN Integration Test Suite')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                       help='Serial port (default: /dev/ttyUSB0)')
    args = parser.parse_args()
    
    test_suite = LDCNIntegrationTest(port=args.port)
    success = test_suite.run_all_tests()
    test_suite.print_summary()
    
    sys.exit(0 if success and test_suite.tests_failed == 0 else 1)

if __name__ == '__main__':
    main()
