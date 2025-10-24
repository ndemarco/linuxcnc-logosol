# LinuxCNC LDCN Driver - Setup Instructions

## Current Status

The project structure has been created with:
- Directory structure (src/, examples/, tests/, docs/)
- README.md
- .gitignore  
- ldcn_protocol.h (protocol definitions)

## What's Missing

The following source files need to be added (approximately 3,400 lines of code):

### Core Implementation (src/)
1. **ldcn_protocol.c** (~340 lines) - Protocol command building and parsing
2. **ldcn_serial.h** (~30 lines) - Serial communication interface
3. **ldcn_serial.c** (~280 lines) - Serial port handling
4. **ldcn.c** (~470 lines) - Main HAL userspace component
5. **Makefile** (~60 lines) - Build system

### Examples (examples/)
6. **ldcn_example.hal** (~150 lines) - HAL configuration example
7. **ldcn_mill.ini** (~200 lines) - Complete INI file

### Tests (tests/)
8. **test_protocol.c** (~300 lines) - Unit tests
9. **test_connection.sh** (~100 lines) - Hardware test script

### Documentation (docs/)
10. **QUICKSTART.md** (~400 lines) - Quick start guide
11. **TECHNICAL.md** (~1000 lines) - Technical reference

## How to Complete the Project

### Option 1: Request Files from Claude

Ask Claude to create each file individually:
- "Please create src/ldcn_protocol.c"
- "Please create src/ldcn_serial.c"
- etc.

### Option 2: Access from Previous Session

If you have access to the previous chat session where all files were created,
you can download them from there.

### Option 3: Manual Implementation

Use the LDCN protocol specification from the Logosol documentation to implement:
1. Command packet building (ldcn_protocol.c)
2. Serial communication (ldcn_serial.c)
3. HAL integration (ldcn.c)

## File Priorities

If recreating manually, create in this order:
1. ldcn_protocol.c - Core protocol implementation
2. ldcn_serial.c - Serial communication  
3. ldcn.c - HAL component
4. Makefile - To build and test
5. Examples and documentation

## Quick Reference

### Protocol Overview
- Header: 0xAA
- Commands: 16 commands (0x0-0xF)
- Checksum: 8-bit sum of address + command + data
- Baud rates: 19200 to 1250000

### Key Commands
- 0xF: Hard Reset
- 0x1: Set Address
- 0x2: Define Status
- 0x4: Load Trajectory
- 0x6: Set Gain
- 0x7: Stop Motor

### HAL Pins (per axis)
- Input: enable, position-cmd, velocity-cmd
- Output: position-fb, velocity-fb, fault

## Getting Help

Once files are created, you can:
1. Build: `cd src && make`
2. Test: `./tests/test_connection.sh`
3. Run: `loadusr -W ldcn port=/dev/ttyUSB0 baud=115200 axes=3`

## Contact

- GitHub: https://github.com/ndemarco/linuxcnc-logosol
- LinuxCNC Forum: https://forum.linuxcnc.org
