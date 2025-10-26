# LDCN HAL Component for LinuxCNC

This LinuxCNC HAL component provides an interface to Logosol Distributed Control Network (LDCN) devices, including LS-231SE servo drives and CNC-SK-2310g2 I/O controllers.

## Features

- **Automatic Network Initialization**: Auto-detects current baud rate and initializes all devices
- **High-Speed Communication**: Operates at 125kbps for fast, responsive control
- **Power Monitoring**: Detects machine power-on state from I/O controller
- **Drive Control**: Enable/disable servo amplifiers from LinuxCNC
- **Robust Error Handling**: Monitors communication status and reports errors
- **Diagnostic Utilities**: Comprehensive tools for troubleshooting and testing

## Hardware Requirements

- Logosol LS-231SE servo drives (up to 5)
- Logosol CNC-SK-2310g2 I/O controller (required for power monitoring)
- USB-to-RS485 adapter (e.g., /dev/ttyUSB0)
- Properly wired LDCN network (daisy-chained RS-485)

## Software Requirements

- LinuxCNC 2.8 or later
- Linux with real-time kernel (PREEMPT-RT or RTAI)
- GCC compiler
- LinuxCNC development packages: `sudo apt install linuxcnc-dev`

## Quick Start

### 1. Build and Install

```bash
# Build the component
make

# Install (requires root)
sudo make install

# Make utility scripts executable
make utils
```

### 2. Initialize the Network

Before using LinuxCNC, initialize the LDCN network:

```bash
./utils/ldcn_init.py --port /dev/ttyUSB0
```

This will:
- Auto-detect current baud rate
- Reset and address all devices
- Switch to 125kbps communication
- Configure I/O controller

### 3. Test the Network

Verify everything is working:

```bash
./utils/ldcn_diagnostic.py --port /dev/ttyUSB0 --test-power
```

### 4. Configure LinuxCNC

Add to your HAL file:

```hal
# Load LDCN component
loadrt ldcn port=/dev/ttyUSB0 num_servos=5

# Add to servo thread
addf ldcn.read servo-thread

# Connect power status
net machine-power ldcn.power-on => halui.machine.on

# Connect enable signal
net machine-enabled motion.motion-enabled => ldcn.enable-request

# Connect per-axis enables (example for X, Y, Z)
net xen ldcn.servo.0.enabled => axis.0.amp-enable-out
net yen ldcn.servo.1.enabled => axis.1.amp-enable-out
net zen ldcn.servo.2.enabled => axis.2.amp-enable-out
```

## HAL Interface

### Pins

| Pin Name | Type | Direction | Description |
|----------|------|-----------|-------------|
| `ldcn.power-on` | bit | OUT | TRUE when machine power is on |
| `ldcn.comms-ok` | bit | OUT | TRUE when LDCN communication is working |
| `ldcn.enable-request` | bit | IN | Request to enable servo amplifiers |
| `ldcn.servo.N.enabled` | bit | OUT | Servo N amplifier is enabled (N=0-4) |

### Parameters

| Parameter Name | Type | Access | Description |
|----------------|------|--------|-------------|
| `ldcn.baud-rate` | u32 | RO | Current baud rate |
| `ldcn.comm-errors` | u32 | RO | Communication error count |

### Module Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | /dev/ttyUSB0 | Serial port device |
| `num_servos` | 5 | Number of servo drives (1-5) |
| `target_baud` | 125000 | Target baud rate |

## Utilities

### ldcn_init.py - Network Initialization

Initialize and configure the LDCN network:

```bash
./utils/ldcn_init.py [options]

Options:
  --port PORT         Serial port (default: /dev/ttyUSB0)
  --num-servos N      Number of servos (default: 5)
  --target-baud BAUD  Target baud rate (default: 125000)
  --verbose           Show detailed communication

Example:
  ./utils/ldcn_init.py --port /dev/ttyUSB0 --target-baud 125000
```

### ldcn_diagnostic.py - Diagnostic Tool

Test and diagnose LDCN network issues:

```bash
./utils/ldcn_diagnostic.py [options]

Options:
  --port PORT       Serial port (default: /dev/ttyUSB0)
  --verbose         Show detailed communication
  --test-power      Test power-on detection

Example:
  ./utils/ldcn_diagnostic.py --port /dev/ttyUSB0 --test-power
```

### ldcn_monitor.py - Status Monitor

Monitor real-time LDCN network status:

```bash
./utils/ldcn_monitor.py [options]

Options:
  --port PORT       Serial port (default: /dev/ttyUSB0)
  --baud BAUD       Baud rate (default: auto-detect)
  --interval SEC    Update interval (default: 1.0)

Example:
  ./utils/ldcn_monitor.py --port /dev/ttyUSB0 --interval 0.5
```

## Typical Usage Workflow

1. **First-time setup**:
   ```bash
   make && sudo make install
   make utils
   ./utils/ldcn_init.py
   ```

2. **Before each LinuxCNC session**:
   - Power on the machine controller
   - Press the physical power button
   - Start LinuxCNC
   - The HAL component will detect power-on state automatically

3. **During operation**:
   - Monitor status: `./utils/ldcn_monitor.py` (optional)
   - Use LinuxCNC normally
   - Enable/disable handled automatically

4. **Troubleshooting**:
   ```bash
   # Test communication
   ./utils/ldcn_diagnostic.py --verbose
   
   # Re-initialize if needed
   ./utils/ldcn_init.py
   ```

## Troubleshooting

See [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for detailed troubleshooting steps.

### Common Issues

**No devices found**:
- Check USB-RS485 adapter connection
- Verify wiring (A, B, GND connections)
- Try different baud rates: `ldcn_diagnostic.py --verbose`

**Power not detected**:
- Ensure I/O controller (device 6) is responding
- Verify J21 jumper configuration on 2310g2
- Test with: `ldcn_diagnostic.py --test-power`

**Communication errors**:
- Check RS-485 termination resistors
- Verify cable quality and length (<1000ft for 125kbps)
- Monitor error count: `halcmd show param ldcn.comm-errors`

## Architecture

The LDCN HAL component consists of three layers:

1. **Protocol Layer** (`ldcn_protocol.c/h`): Low-level LDCN serial communication
2. **HAL Layer** (`ldcn_hal.c`): LinuxCNC real-time component interface
3. **Utilities** (Python scripts): Configuration and diagnostic tools

## Network Topology

```
┌─────────────┐
│  LinuxCNC   │
│  (PC)       │
└─────┬───────┘
      │ USB
┌─────▼─────────┐
│  USB-RS485    │
│  Converter    │
└─────┬─────────┘
      │ RS-485 (A, B, GND)
      │
┌─────▼─────────┐    ┌──────────────┐    ┌──────────────┐
│ Servo Drive 1 ├───►│Servo Drive 2 ├───►│Servo Drive 3 │ ...
│  (Addr 1)     │    │  (Addr 2)    │    │  (Addr 3)    │
└───────────────┘    └──────────────┘    └──────┬───────┘
                                                 │
                                          ┌──────▼─────────┐
                                          │  I/O Controller│
                                          │  (Addr 6)      │
                                          └────────────────┘
```

## Protocol Details

See [PROTOCOL.md](docs/PROTOCOL.md) for complete LDCN protocol documentation.

## Development

### Building from Source

```bash
# Clone repository
git clone https://github.com/ndemarco/linuxcnc-logosol.git
cd linuxcnc-logosol

# Build
make

# Install
sudo make install
```

### Testing

```bash
# Run protocol tests
make test

# Run integration tests
cd tests
python3 integration_test.py
```

## License

GPL v2 or later

## Support

- GitHub Issues: https://github.com/ndemarco/linuxcnc-logosol/issues
- LinuxCNC Forum: https://forum.linuxcnc.org/

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes with tests
4. Submit a pull request

## References

- [LinuxCNC HAL Manual](http://linuxcnc.org/docs/html/hal/intro.html)
- [Logosol LS-231SE Documentation](docs/LS231SEAdvancedMultifunctionalServoDrive.pdf)
- [Logosol CNC-SK-2310g2 Documentation](docs/LS2310g2SupervisorIOController.pdf)

## Credits

Developed by the LinuxCNC community for Logosol CNC machines.

Special thanks to:
- Logosol Inc. for LDCN protocol documentation
- LinuxCNC developers for the HAL framework
