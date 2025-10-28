# Git Commit Instructions

This file contains instructions for committing the LDCN HAL component code to the repository.

## Initial Setup

```bash
cd /path/to/linuxcnc-logosol
git checkout -b feature/ldcn-hal-component
```

## Commit Sequence

Follow this sequence to create logical, reviewable commits:

### Commit 1: Protocol Layer

```bash
git add src/ldcn_protocol.h src/ldcn_protocol.c
git commit -m "Add LDCN protocol implementation

- Implement core LDCN serial communication protocol
- Support baud rates 9600-1.25Mbps
- Auto-detection of current network state
- Commands: reset, address, status, stop motor, baud change
- Robust error handling and timeouts
- Comprehensive documentation in header"
```

### Commit 2: HAL Component

```bash
git add src/ldcn_hal.c
git commit -m "Add LinuxCNC HAL component for LDCN

- Real-time HAL component for Logosol LDCN networks
- Automatic network initialization at startup
- Power-on detection from 2310g2 I/O controller
- Servo amplifier enable/disable control
- Communication status monitoring
- Support for 5 servo drives + 1 I/O controller"
```

### Commit 3: Build System

```bash
git add Makefile
git commit -m "Add Makefile for building LDCN HAL component

- Build HAL shared library
- Install/uninstall targets
- Test target
- Auto-detect LinuxCNC installation
- Clean build support"
```

### Commit 4: Initialization Utility

```bash
git add utils/ldcn_init.py
chmod +x utils/ldcn_init.py
git add utils/ldcn_init.py
git commit -m "Add LDCN network initialization utility

- Auto-detect current baud rate
- Reset and address all devices
- Switch to target baud rate (default 125kbps)
- Configure I/O controller for full status
- Comprehensive error handling and logging"
```

### Commit 5: Diagnostic Utility

```bash
git add utils/ldcn_diagnostic.py
chmod +x utils/ldcn_diagnostic.py
git add utils/ldcn_diagnostic.py
git commit -m "Add LDCN network diagnostic utility

- Auto-detect and test all devices
- Verify communication at any baud rate
- Test power-on detection functionality
- Verbose mode for debugging
- User-friendly output and error messages"
```

### Commit 6: Monitor Utility

```bash
git add utils/ldcn_monitor.py
chmod +x utils/ldcn_monitor.py
git add utils/ldcn_monitor.py
git commit -m "Add LDCN network status monitor

- Real-time monitoring of all devices
- Display power state and status bytes
- Configurable update interval
- Color-coded status display
- Auto-detect baud rate"
```

### Commit 7: Integration Tests

```bash
git add tests/integration_test.py
chmod +x tests/integration_test.py
git add tests/integration_test.py
git commit -m "Add LDCN integration test suite

- Comprehensive test coverage
- Auto-detection test
- Network initialization test
- Baud rate change test
- Power detection test
- Servo enable/disable test
- Color-coded pass/fail output
- Test summary with statistics"
```

### Commit 8: Documentation

```bash
git add README.md
git commit -m "Add main README documentation

- Quick start guide
- Installation instructions
- HAL interface reference
- Utility documentation
- Usage workflow
- Architecture overview
- Network topology diagram"
```

### Commit 9: Protocol Documentation

```bash
git add docs/PROTOCOL.md
git commit -m "Add LDCN protocol documentation

- Complete protocol specification
- Packet structure definitions
- Command reference
- Status byte descriptions
- Baud rate divisors table
- Initialization sequence
- Timing requirements
- Example communication session
- Wiring diagrams"
```

### Commit 10: Troubleshooting Guide

```bash
git add docs/TROUBLESHOOTING.md
git commit -m "Add LDCN troubleshooting guide

- Common issues and solutions
- Diagnostic procedures
- Hardware debugging steps
- Advanced debugging techniques
- Known issues and workarounds
- Preventive maintenance tips"
```

## Push and Create PR

```bash
# Push feature branch
git push origin feature/ldcn-hal-component

# Create pull request on GitHub
# Title: "Add LDCN HAL Component for Logosol CNC Machines"
# Description: See PR_DESCRIPTION.md below
```

## Pull Request Description Template

```markdown
# Add LDCN HAL Component for Logosol CNC Machines

This PR adds a complete HAL component for controlling Logosol Distributed Control Network (LDCN) devices from LinuxCNC.

## Features

- **Automatic initialization**: Auto-detects baud rate, resets network, addresses devices
- **High-speed communication**: Operates at 125kbps for responsive control
- **Power monitoring**: Detects machine power-on from 2310g2 I/O controller
- **Drive control**: Enable/disable servo amplifiers from LinuxCNC
- **Robust error handling**: Monitors communication status and reports errors
- **Comprehensive utilities**: Diagnostic, initialization, and monitoring tools

## Hardware Support

- LS-231SE servo drives (up to 5)
- CNC-SK-2310g2 I/O controller

## Testing

- Tested on real hardware with 5 servos + I/O controller
- Integration test suite included
- All utilities tested and verified

## Documentation

- Complete README with quick start guide
- Detailed protocol documentation
- Comprehensive troubleshooting guide
- Example HAL configurations

## Files Added

- `src/ldcn_protocol.h/c` - LDCN protocol implementation
- `src/ldcn_hal.c` - LinuxCNC HAL component
- `Makefile` - Build system
- `utils/` - Utility scripts (init, diagnostic, monitor)
- `tests/` - Integration test suite
- `docs/` - Complete documentation

## Breaking Changes

None - this is a new component

## Migration Path

Not applicable - new component

## Review Notes

- All code follows LinuxCNC coding standards
- Comprehensive error handling throughout
- Well-documented with inline comments
- Utilities follow Unix philosophy (do one thing well)
- Tested on real hardware

## Checklist

- [x] Code builds without warnings
- [x] All utilities tested
- [x] Documentation complete
- [x] Integration tests pass
- [x] No memory leaks
- [x] Follows coding standards
```

## Verification Before Push

Before pushing, verify:

```bash
# Check all files are added
git status

# Verify each commit
git log --oneline

# Build test
make clean && make

# Run tests
chmod +x utils/*.py tests/*.py
./tests/integration_test.py

# Check no sensitive data
git diff origin/main
```

## Alternative: Squash Commits

If you prefer a single commit:

```bash
git add .
git commit -m "Add complete LDCN HAL component with utilities and documentation

Complete implementation of LDCN HAL component for LinuxCNC including:
- Protocol layer with full LDCN communication support
- HAL component for real-time control
- Initialization, diagnostic, and monitoring utilities  
- Integration test suite
- Comprehensive documentation

Supports LS-231SE servo drives and CNC-SK-2310g2 I/O controller.
Tested on real hardware with 5 servos + I/O controller."
```

## Post-Merge

After merge:

1. Update documentation website
2. Announce on LinuxCNC forum
3. Create release tag
4. Update project wiki

## Need Help?

If you need help with git or the PR process:
- LinuxCNC Developer Guide: https://linuxcnc.org/docs/devel/html/
- Git Tutorial: https://git-scm.com/docs/gittutorial
- GitHub PR Guide: https://docs.github.com/en/pull-requests
