# Python File Inventory

**Date:** 2025-10-29
**Location:** `/home/nick/projects/linuxcnc-logosol/utils/`

## Source Files for Refactoring

### 1. ldcn_diagnostic.py (11,416 bytes)
**Purpose:** LDCN Network Diagnostic Utility

**Key Features:**
- Auto-detecting current baud rate
- Testing communication with all devices
- Displaying device status and fault conditions
- Verifying power control functionality
- Monitoring servo faults (checksum errors, current limit, position error)
- Continuous fault monitoring

**Class:**
- `LDCNDiagnostic`

**Key Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
- `try_baud(baud)` - Try communication at specific baud rate
- `auto_detect_baud()` - Auto-detect current baud rate
- `check_faults(status)` - Check status byte for fault conditions
- `decode_status(status)` - Decode status byte into human-readable flags
- `test_device(address, name)` - Test communication with single device
- `test_all_devices()` - Test all expected devices
- `test_faults()` - Continuously monitor servos for faults
- `test_power_detection()` - Test power-on detection from I/O controller
- `run(test_power, test_faults)` - Main diagnostic runner

**Status Bit Flags Defined:**
- `STATUS_MOVE_DONE = (1 << 0)`
- `STATUS_CKSUM_ERROR = (1 << 1)`
- `STATUS_CURRENT_LIMIT = (1 << 2)`
- `STATUS_POWER_ON = (1 << 3)`
- `STATUS_POS_ERROR = (1 << 4)`
- `STATUS_HOME_SOURCE = (1 << 5)`
- `STATUS_LIMIT2 = (1 << 6)`
- `STATUS_HOME_IN_PROG = (1 << 7)`

---

### 2. ldcn_init.py (9,899 bytes)
**Purpose:** LDCN Network Initialization Utility

**Key Features:**
- Auto-detecting and resetting network
- Addressing all devices sequentially
- Switching to target baud rate
- Configuring I/O controller

**Class:**
- `LDCNInitializer`

**Key Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
- `open_serial(baud)` - Open serial port at specified baud rate
- `auto_detect_baud()` - Auto-detect current baud rate
- `hard_reset()` - Send hard reset command
- `address_devices()` - Address all devices sequentially
- `verify_devices()` - Verify all devices are responding
- `change_baud_rate(new_baud)` - Change baud rate of all devices
- `configure_io_controller()` - Configure I/O controller for full status
- `initialize()` - Complete initialization sequence

**Baud Rate Definitions:**
```python
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
```

---

### 3. ldcn_monitor.py (7,038 bytes)
**Purpose:** LDCN Network Status Monitor

**Key Features:**
- Continuous monitoring of LDCN network status
- Communication status with all devices
- Power-on state from I/O controller
- Real-time status updates

**Class:**
- `LDCNMonitor`

**Key Methods:**
- `send_command(address, command, data=[])` - Send LDCN command
- `auto_detect_baud()` - Auto-detect current baud rate
- `configure_io_controller()` - Configure I/O controller for full status
- `read_device_status(address)` - Read status from a device
- `format_status_bits(status)` - Format status byte as bit string
- `print_header()` - Print monitor header
- `print_device_status(address, info)` - Print status for one device
- `monitor()` - Main monitoring loop

**Device Names:**
```python
device_names = {
    1: "Servo 1",
    2: "Servo 2",
    3: "Servo 3",
    4: "Servo 4",
    5: "Servo 5",
    6: "I/O Ctrl"
}
```

---

### 4. test_servo_init.py (11,574 bytes)
**Purpose:** Servo Initialization Testing (matches Python script used during debugging)

**Key Features:**
- Tests servo drive initialization
- Verifies communication
- Tests position reading
- Matches exact termios settings from C code

**Functions:** (To be extracted)

---

### 5. test_position_command.py (4,745 bytes)
**Purpose:** Position Command Testing

**Key Features:**
- Tests sending position commands to servos
- Verifies motion control
- Tests trajectory commands

**Functions:** (To be extracted)

---

## Common Protocol Constants

**Header:**
- `HEADER = 0xAA`

**Commands:**
- `CMD_NOP = 0x0E` - No operation / status read
- `CMD_HARD_RESET = 0x0F` - Hard reset all devices
- `CMD_SET_ADDRESS = 0x01` - Set device address
- `CMD_SET_BAUD = 0x0A` - Set baud rate
- `CMD_DEFINE_STATUS = 0x02` - Define status reporting

**Common Baud Rates:**
- `[19200, 125000, 115200, 57600, 9600, 38400]`

**Default Devices:**
- Addresses 1-5: Servo drives (LS-231SE)
- Address 6: I/O Controller (SK-2310g2)

---

## Next Steps

1. Extract all functions from test_servo_init.py
2. Extract all functions from test_position_command.py
3. Categorize all functions by type
4. Begin architecture design

---

## Notes

- All utilities use similar serial communication patterns
- Common command building: `(num_data << 4) | (command & 0x0F)`
- Checksum calculation: `(address + cmd_byte + sum(data)) & 0xFF`
- All use 0.02s delay after sending commands
- All use similar auto-detection logic
