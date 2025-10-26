# LinuxCNC Logosol LDCN Driver

A non-realtime HAL driver for Logosol LDCN servo drives (LS-231SE, LS-2310g2) with LinuxCNC.

## Overview

This driver provides LinuxCNC integration with Logosol Distributed Control Network (LDCN) servo drives. It communicates over RS-485 via USB using the LS-832RL adapter and supports:

- Multi-axis coordinated motion (up to 31 axes)
- Non-realtime operation (no PREEMPT_RT kernel required)
- Position, velocity, and acceleration control
- Status monitoring and fault detection
- Configurable servo update rates

## Hardware Requirements

- Logosol LS-231SE or LS-2310g2 servo drives
- LS-832RL USB to LDCN converter (FTDI FT232R-based)
- Standard RJ-45 cables for daisy-chaining drives

## Quick Start

```bash
# Build
cd src && make

# Install
sudo make install

# Load driver
loadusr -W ldcn port=/dev/ttyUSB0 baud=115200 axes=3
```

## Documentation

- **QUICKSTART.md** - Get running in 10 minutes
- **TECHNICAL.md** - Detailed technical reference
- **examples/** - Sample configurations

## License

GPL-2.0 (compatible with LinuxCNC)

## Author

Created for the LinuxCNC community.
