# LDCN Python Module Refactoring Plan

**Goal:** Create a reusable Python module for LDCN network communication with object-oriented device abstraction.

**Start Date:** 2025-10-29

---

## Architecture Overview

```
LDCNNetwork (top-level class)
├── Serial port management
├── Protocol-level operations (reset, baud, addressing)
├── Device discovery and management
└── devices[] - list of LDCNDevice instances

LDCNDevice (abstract base class)
├── Common properties: address, network ref, device_type
├── Common methods: send_command(), read_status(), verify_checksum()
└── Protocol compliance

LS231SE (servo drive subclass)
├── Servo-specific properties: position, velocity, servo_on, fault
├── Servo methods: enable(), disable(), move_to(), set_gains(), home()
└── Status monitoring

SK2310g2 (supervisor subclass)
├── Supervisor properties: diagnostic_code, digital_inputs, power_state
├── Supervisor methods: read_diagnostic(), read_inputs(), set_outputs()
└── Safety monitoring
```

---

## Phase 1: Survey (Understanding Existing Code)

### Task 1.1: Clean up directory structure
**Status:** ✅ COMPLETED
**Actions Taken:**
- Merged `utilities/` into `utils/`
- Kept `utils/` (has superior ldcn_diagnostic.py with fault monitoring)
- Moved `test_position_command.py` to `utils/`
- Moved `test_servo_init.py` to `utils/`
- Deleted duplicate `utilities/` directory

**Result:** All Python utilities now in `/home/nick/projects/linuxcnc-logosol/utils/`

### Task 1.2: Create Python file inventory
**Status:** ✅ COMPLETED
**Output:** `PYTHON_FILE_INVENTORY.md` created with:
- 5 Python files documented
- Key classes and methods identified
- Common protocol constants extracted
- Device addressing scheme documented

**Files:**
1. `ldcn_diagnostic.py` - Diagnostic utility with fault monitoring
2. `ldcn_init.py` - Network initialization
3. `ldcn_monitor.py` - Real-time status monitoring
4. `test_servo_init.py` - Servo initialization testing
5. `test_position_command.py` - Position command testing

### Task 1.3: Extract functions from test files
**Status:** Pending
**Method:** Read test_servo_init.py and test_position_command.py, extract all functions
**Output:** Add to function inventory

### Task 1.4: Categorize functions
**Status:** Pending
**Categories:**
- **Protocol-level:** Serial communication, checksums, packet building
- **Servo-specific:** Position control, gain setting, status reading
- **Supervisor-specific:** Diagnostic reading, I/O control
- **Utility:** Helpers, formatters, converters

**Output:** Categorized function map

---

## Phase 2: Design (Planning Architecture)

### Task 2.1: Define LDCNNetwork class interface
**Status:** Pending
**Deliverable:** Class definition with method signatures, properties, docstrings

```python
class LDCNNetwork:
    """LDCN network manager for multi-drop serial communication"""

    # Properties
    port: str
    baud_rate: int
    devices: List[LDCNDevice]

    # Methods
    def open() -> None
    def close() -> None
    def reset() -> None
    def set_baud(baud: int) -> None
    def discover_devices() -> List[LDCNDevice]
```

### Task 2.2: Define LDCNDevice base class interface
**Status:** Pending
**Deliverable:** Abstract base class definition

```python
class LDCNDevice(ABC):
    """Base class for all LDCN devices"""

    # Properties
    address: int
    network: LDCNNetwork
    device_type: str
    model_id: str

    # Abstract methods
    @abstractmethod
    def read_status() -> dict

    # Common methods
    def send_command(cmd: int, data: bytes) -> bytes
    def verify_checksum(packet: bytes) -> bool
```

### Task 2.3: Define device-specific subclass interfaces
**Status:** Pending
**Deliverable:** LS231SE and SK2310g2 class definitions

### Task 2.4: Map existing functions to class methods
**Status:** Pending
**Deliverable:** Mapping table showing old function → new method

---

## Phase 3: Implementation (Building the Module)

### Task 3.1: Create ldcn_network.py module
**Status:** Pending
**Location:** `/home/nick/projects/linuxcnc-logosol/ldcn_network.py`
**Contents:**
- LDCNNetwork class
- LDCNDevice base class
- Device subclasses (LS231SE, SK2310g2)

### Task 3.2: Refactor protocol functions into LDCNNetwork
**Status:** Pending
**Focus:** Serial I/O, reset, baud setting, addressing

### Task 3.3: Refactor servo functions into LS231SE
**Status:** Pending
**Focus:** Motion control, status reading, gain setting

### Task 3.4: Refactor supervisor functions into SK2310g2
**Status:** Pending
**Focus:** Diagnostic reading, I/O control

### Task 3.5: Add docstrings and type hints
**Status:** Pending
**Requirements:**
- All classes have docstrings
- All methods have type hints
- All parameters documented

---

## Phase 4: Validation (Testing)

### Task 4.1: Create test script
**Status:** Pending
**Location:** `/home/nick/projects/linuxcnc-logosol/test_ldcn_module.py`
**Tests:**
- Network initialization
- Device discovery
- Servo commands
- Supervisor queries

### Task 4.2: Test device discovery
**Status:** Pending
**Verify:** Correct devices found at correct addresses

### Task 4.3: Test servo operations
**Status:** Pending
**Verify:** Position read/write, enable/disable, status monitoring

### Task 4.4: Test supervisor operations
**Status:** Pending
**Verify:** Diagnostic reading matches LED display

### Task 4.5: Compare with original scripts
**Status:** Pending
**Verify:** New module produces same results as original utilities

---

## Target Usage Example

```python
#!/usr/bin/env python3
from ldcn_network import LDCNNetwork

# Initialize network
net = LDCNNetwork('/dev/ttyUSB0')
net.open()
net.discover_devices()

# Access supervisor
supervisor = net.devices[6]  # SK2310g2 at address 6
diagnostic = supervisor.read_diagnostic()
print(f"Supervisor diagnostic: 0x{diagnostic:02X} - {supervisor.diagnostic_description}")

# Access servo drive
servo_y = net.devices[1]  # LS231SE Y-axis at address 1
position = servo_y.read_position()
print(f"Y-axis position: {position}")

# Move servo
servo_y.enable()
servo_y.move_to(position + 1000, velocity=100)

# Clean up
net.close()
```

---

## Decision Log

### Decision 1: Use single file vs package
**Date:** TBD
**Decision:** TBD
**Rationale:** TBD

### Decision 2: Backwards compatibility
**Date:** TBD
**Decision:** TBD
**Rationale:** Should old scripts continue to work?

### Decision 3: Error handling strategy
**Date:** TBD
**Decision:** TBD
**Rationale:** Exceptions vs return codes?

---

## Notes

- Keep original utility scripts intact during development
- Test on real hardware frequently
- Document any protocol discoveries
- Compare serial communication byte-for-byte with working Python scripts

---

## Progress Tracking

**Phase 1:** 🟡 In Progress (Task 1.1 ✅, Task 1.2 ✅, Task 1.3 pending)
**Phase 2:** Not started
**Phase 3:** Not started
**Phase 4:** Not started

**Last Updated:** 2025-10-29

---

## Change Log

### 2025-10-29 08:25
- ✅ Merged utilities/ into utils/ directory
- ✅ Moved test files to utils/
- ✅ Created PYTHON_FILE_INVENTORY.md
- 📝 Identified 5 Python files for refactoring
- 📝 Documented common protocol patterns
