# LDCN Function Categorization

**Date:** 2025-10-29
**Purpose:** Categorize all functions from existing utilities for refactoring into class-based architecture

---

## Category 1: Protocol-Level Functions (LDCNNetwork)

These handle low-level serial communication, packet building, and network management.

### Serial Communication
- `send_command(ser, address, command, data=[])`
  - **Used in:** ALL files
  - **Function:** Build and send LDCN command packet, receive response
  - **Signature:** `(serial, int, int, list[int]) -> bytes`
  - **Target:** `LDCNNetwork.send_command(address, command, data) -> bytes`

### Baud Rate Management
- `try_baud(baud)` / `try_communicate_at_baud(port, baud)`
  - **Used in:** ldcn_diagnostic.py, ldcn_init.py, test_servo_init.py
  - **Function:** Test if devices respond at specific baud rate
  - **Target:** `LDCNNetwork._try_baud(baud) -> bool`

- `auto_detect_baud()` / `find_current_baud(port)`
  - **Used in:** ldcn_diagnostic.py, ldcn_init.py, ldcn_monitor.py, test_servo_init.py
  - **Function:** Auto-detect current network baud rate
  - **Target:** `LDCNNetwork.auto_detect_baud() -> int`

- `change_baud_rate(new_baud)`
  - **Used in:** ldcn_init.py
  - **Function:** Change baud rate of all devices on network
  - **Target:** `LDCNNetwork.set_baud_rate(baud) -> bool`

- `open_serial(baud)`
  - **Used in:** ldcn_init.py
  - **Function:** Open serial port at specific baud rate
  - **Target:** `LDCNNetwork._open_port(baud) -> None`

### Network Initialization
- `hard_reset()`
  - **Used in:** ldcn_init.py, test_servo_init.py
  - **Function:** Send hard reset command to all devices (0xFF, 0x0F)
  - **Target:** `LDCNNetwork.reset() -> None`

- `address_devices()`
  - **Used in:** ldcn_init.py, test_servo_init.py
  - **Function:** Sequentially address all devices (SET_ADDRESS command)
  - **Target:** `LDCNNetwork.address_devices(num_devices) -> int`

- `verify_devices()`
  - **Used in:** ldcn_init.py
  - **Function:** Verify all devices are responding with NOP command
  - **Target:** `LDCNNetwork.verify_devices() -> int`

### Device Discovery
- `test_device(address, name)`
  - **Used in:** ldcn_diagnostic.py
  - **Function:** Test communication with single device
  - **Target:** `LDCNNetwork.test_device(address) -> bool`

- `test_all_devices()`
  - **Used in:** ldcn_diagnostic.py
  - **Function:** Test all expected devices (1-6)
  - **Target:** `LDCNNetwork.discover_devices() -> list[LDCNDevice]`

### Network Management
- `initialize()`
  - **Used in:** ldcn_init.py
  - **Function:** Complete initialization sequence (reset, address, baud, configure)
  - **Target:** `LDCNNetwork.initialize(target_baud, num_servos) -> bool`

---

## Category 2: Servo-Specific Functions (LS231SE class)

These handle servo drive operations - motion control, status reading, configuration.

### Status Reading & Parsing
- `parse_servo_status(response, status_bits)`
  - **Used in:** test_servo_init.py
  - **Function:** Parse variable-length servo status response
  - **Target:** `LS231SE._parse_status(response) -> dict`

- `read_drive_position(ser, addr)`
  - **Used in:** test_position_command.py
  - **Function:** Read current position and status from drive
  - **Target:** `LS231SE.read_position() -> dict`

- `decode_status(status)`
  - **Used in:** ldcn_diagnostic.py
  - **Function:** Decode status byte into human-readable flags
  - **Target:** `LS231SE.decode_status_flags(status_byte) -> list[str]`

### Fault Detection
- `check_faults(status)`
  - **Used in:** ldcn_diagnostic.py
  - **Function:** Check status byte for fault conditions (CKSUM_ERR, CURRENT_LIM, POS_ERR)
  - **Target:** `LS231SE.check_faults(status_byte) -> list[str]`

### Servo Initialization
- `initialize_servo(ser, addr)`
  - **Used in:** test_servo_init.py
  - **Function:** Complete 7-step servo initialization sequence
  - **Target:** `LS231SE.initialize() -> bool`
  - **Steps:**
    1. Define status reporting (DEFINE_STATUS)
    2. Set PID gains (LOAD_GAINS)
    3. Load initial trajectory (LOAD_TRAJECTORY)
    4. Enable amplifier (STOP_MOTOR with AMP_ENABLE)
    5. Reset position counter (RESET_POS)
    6. Clear sticky status bits (CLEAR_BITS)
    7. Read and verify status (READ_STATUS)

### Motion Control
- `send_position_command(ser, addr, position_counts)`
  - **Used in:** test_position_command.py
  - **Function:** Send position trajectory command
  - **Target:** `LS231SE.move_to(position, velocity, accel) -> bool`

### Configuration
- PID gain setting (embedded in initialize_servo)
  - **Target:** `LS231SE.set_gains(kp, kd, ki, il, ol, cl, el, sr, db) -> bool`

- Status reporting configuration (embedded in initialize_servo)
  - **Target:** `LS231SE.configure_status(status_bits) -> None`

---

## Category 3: Supervisor-Specific Functions (SK2310g2 class)

These handle supervisor I/O controller operations - diagnostic reading, power monitoring.

### Configuration
- `configure_io_controller()`
  - **Used in:** ldcn_init.py, ldcn_monitor.py, test_servo_init.py
  - **Function:** Configure supervisor for full status reporting (DEFINE_STATUS 0xFF, 0xFF)
  - **Target:** `SK2310g2.configure() -> None`

### Diagnostic Reading
- `read_device_status(address)` (when address == 6)
  - **Used in:** ldcn_monitor.py
  - **Function:** Read supervisor status including diagnostic code
  - **Target:** `SK2310g2.read_diagnostic() -> int`

### Power Detection
- `test_power_detection()`
  - **Used in:** ldcn_diagnostic.py, test_servo_init.py (embedded in main)
  - **Function:** Monitor supervisor for power button press
  - **Target:** `SK2310g2.wait_for_power_button(timeout) -> bool`

### Digital I/O
- Power state monitoring (embedded in various functions)
  - **Target:** `SK2310g2.read_power_state() -> bool`
  - **Target:** `SK2310g2.read_digital_inputs() -> int`

---

## Category 4: Utility Functions

These provide helper functionality - logging, formatting, display.

### Logging
- `log(msg, level='INFO')`
  - **Used in:** ldcn_diagnostic.py, ldcn_init.py
  - **Function:** Print log message with level
  - **Target:** Python `logging` module or class-level logger

### Formatting
- `format_status_bits(status)`
  - **Used in:** ldcn_monitor.py
  - **Function:** Format status byte as binary string
  - **Target:** Utility method or property

### Display/Monitoring
- `print_header()`
  - **Used in:** ldcn_monitor.py
  - **Function:** Print monitor header with timestamp
  - **Target:** Remove (UI-specific, not in module)

- `print_device_status(address, info)`
  - **Used in:** ldcn_monitor.py
  - **Function:** Print formatted device status
  - **Target:** Remove (UI-specific, not in module)

### Fault Monitoring
- `test_faults()`
  - **Used in:** ldcn_diagnostic.py
  - **Function:** Continuously monitor all servos for faults
  - **Target:** `LDCNNetwork.monitor_faults(duration, callback) -> int`

### Main/Run Functions
- `run()` - ldcn_diagnostic.py
- `initialize()` - ldcn_init.py
- `monitor()` - ldcn_monitor.py
- `main()` - test scripts

**Target:** These become example scripts that use the new module, not part of the module itself

---

## Function Mapping Summary

### LDCNNetwork Class
| Method | Source Function(s) | Purpose |
|--------|-------------------|---------|
| `__init__(port, baud=None)` | New | Initialize network |
| `open()` | `open_serial()` | Open serial port |
| `close()` | New | Close serial port |
| `send_command()` | `send_command()` | Send command to any device |
| `auto_detect_baud()` | `auto_detect_baud()` / `find_current_baud()` | Detect current baud |
| `set_baud_rate()` | `change_baud_rate()` | Change network baud |
| `reset()` | `hard_reset()` | Hard reset all devices |
| `address_devices()` | `address_devices()` | Address all devices |
| `discover_devices()` | `test_all_devices()` | Discover and create device objects |
| `initialize()` | `initialize()` from ldcn_init.py | Full init sequence |

### LDCNDevice Base Class
| Method | Source | Purpose |
|--------|--------|---------|
| `__init__(network, address)` | New | Base device init |
| `send_command()` | Delegates to network | Send command to this device |
| `read_status()` | Abstract | Device-specific status read |

### LS231SE Class (extends LDCNDevice)
| Method | Source Function(s) | Purpose |
|--------|-------------------|---------|
| `initialize()` | `initialize_servo()` | 7-step init sequence |
| `read_status()` | `parse_servo_status()` | Read detailed status |
| `read_position()` | `read_drive_position()` | Read position only |
| `set_gains()` | Extracted from `initialize_servo()` | Set PID gains |
| `move_to()` | `send_position_command()` | Position command |
| `enable()` | Extracted from `initialize_servo()` | Enable amplifier |
| `disable()` | New | Disable amplifier |
| `reset_position()` | Extracted from `initialize_servo()` | Reset position counter |
| `clear_faults()` | Extracted from `initialize_servo()` | Clear sticky bits |
| `check_faults()` | `check_faults()` | Check for fault conditions |

### SK2310g2 Class (extends LDCNDevice)
| Method | Source Function(s) | Purpose |
|--------|-------------------|---------|
| `configure()` | `configure_io_controller()` | Configure for full status |
| `read_diagnostic()` | `read_device_status()` | Read diagnostic code |
| `read_power_state()` | Extracted | Check power button state |
| `wait_for_power_button()` | `test_power_detection()` | Wait for power press |
| `read_digital_inputs()` | New | Read digital inputs |
| `set_digital_outputs()` | New | Set digital outputs |

---

## Protocol Constants to Include

```python
# Commands
CMD_RESET_POS = 0x00
CMD_SET_ADDRESS = 0x01
CMD_DEFINE_STATUS = 0x02
CMD_READ_STATUS = 0x03
CMD_LOAD_TRAJECTORY = 0x04
CMD_START_MOTION = 0x05
CMD_LOAD_GAINS = 0x06
CMD_STOP_MOTOR = 0x07
CMD_SET_BAUD = 0x0A
CMD_CLEAR_BITS = 0x0B
CMD_NOP = 0x0E
CMD_HARD_RESET = 0x0F

# Status Bits
STATUS_MOVE_DONE = 0x01
STATUS_CKSUM_ERROR = 0x02
STATUS_CURRENT_LIMIT = 0x04
STATUS_POWER_ON = 0x08
STATUS_POS_ERROR = 0x10
STATUS_HOME_SOURCE = 0x20
STATUS_LIMIT2 = 0x40
STATUS_HOME_IN_PROG = 0x80

# Baud Rate Values
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

## Next Steps

1. Design detailed class interfaces with type hints
2. Create ldcn_network.py module skeleton
3. Begin implementation starting with LDCNNetwork base functionality
4. Add device-specific subclasses
5. Create test/example scripts
