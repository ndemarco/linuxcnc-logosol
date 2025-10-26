# LDCN HAL Component for LinuxCNC

This LinuxCNC HAL component provides an interface to Logosol Distributed Control Network (LDCN) devices, including LS-231SE servo drives and CNC-SK-2310g2 I/O controllers.

## Features

- **Automatic Network Initialization**: Auto-detects current baud rate and initializes all devices
- **High-Speed Communication**: Operates at 125kbps for fast, responsive control
- **Power Monitoring**: Detects machine power-on state from I/O controller
- **Drive Control**: Enable/disable servo amplifiers from LinuxCNC
- **Emergency Stop**: Dedicated E-stop input immediately disables all servos
- **Fault Detection**: Monitors servos for checksum errors, current limit, and position errors
- **Watchdog Timer**: Auto-disables servos on communication timeout (configurable)
- **Auto-Safety Disable**: Automatically disables on power loss or fault conditions
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

# Configure watchdog timeout (optional, default is 1.0 second)
setp ldcn.watchdog-timeout 0.5

# Connect power status
net machine-power ldcn.power-on => halui.machine.on

# Connect emergency stop
net estop-out iocontrol.0.user-enable-out => ldcn.estop

# Connect enable signal
net machine-enabled motion.motion-enabled => ldcn.enable-request

# Connect fault signals
net ldcn-fault ldcn.fault => halui.estop.is-activated

# Connect per-axis enables (example for X, Y, Z)
net xen ldcn.servo.0.enabled => axis.0.amp-enable-out
net yen ldcn.servo.1.enabled => axis.1.amp-enable-out
net zen ldcn.servo.2.enabled => axis.2.amp-enable-out

# Optional: Monitor individual servo faults
net x-fault ldcn.servo.0.fault => halui.estop.is-activated
net y-fault ldcn.servo.1.fault => halui.estop.is-activated
net z-fault ldcn.servo.2.fault => halui.estop.is-activated
```

## HAL Interface

### Pins

| Pin Name | Type | Direction | Description |
|----------|------|-----------|-------------|
| `ldcn.power-on` | bit | OUT | TRUE when machine power is on |
| `ldcn.comms-ok` | bit | OUT | TRUE when LDCN communication is working |
| `ldcn.fault` | bit | OUT | TRUE when any servo has a fault |
| `ldcn.enable-request` | bit | IN | Request to enable servo amplifiers |
| `ldcn.estop` | bit | IN | Emergency stop input (TRUE = stop all servos) |
| `ldcn.servo.N.enabled` | bit | OUT | Servo N amplifier is enabled (N=0-4) |
| `ldcn.servo.N.fault` | bit | OUT | Servo N has a fault condition (N=0-4) |

### Parameters

| Parameter Name | Type | Access | Description |
|----------------|------|--------|-------------|
| `ldcn.baud-rate` | u32 | RO | Current baud rate |
| `ldcn.comm-errors` | u32 | RO | Communication error count |
| `ldcn.watchdog-timeout` | float | RW | Watchdog timeout in seconds (default: 1.0) |

### Module Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | /dev/ttyUSB0 | Serial port device |
| `num_servos` | 5 | Number of servo drives (1-5) |
| `target_baud` | 125000 | Target baud rate |

## Safety Features

The LDCN HAL component includes multiple layers of safety protection:

### Emergency Stop

- **Dedicated E-stop Pin**: `ldcn.estop` input immediately disables all servos when asserted
- **Hardware-level Stop**: Sends LDCN stop command with amplifiers disabled and motors off
- **Fast Response**: E-stop is checked first in every servo thread cycle
- **Usage**: Connect to your machine's E-stop chain:
  ```hal
  net estop-out iocontrol.0.user-enable-out => ldcn.estop
  ```

### Fault Detection

The component continuously monitors each servo for fault conditions:
- **Checksum Errors**: Communication data integrity problems
- **Current Limit**: Motor drawing excessive current
- **Position Error**: Following error exceeded

Faults are reported via:
- `ldcn.fault` - Global fault indicator (any servo)
- `ldcn.servo.N.fault` - Per-servo fault status

**Fault Response**: Servos are automatically disabled when faults are detected.

### Watchdog Timer

- **Purpose**: Protects against HAL thread failures or communication freezes
- **Default**: 1.0 second timeout
- **Configurable**: Adjust via `ldcn.watchdog-timeout` parameter
- **Action**: Automatically disables all servos if no successful communication within timeout period
- **Disable**: Set to 0 to disable watchdog (not recommended)

Example configuration:
```hal
setp ldcn.watchdog-timeout 0.5  # 500ms timeout for faster response
```

### Automatic Safety Disables

Servos are automatically disabled when:
1. **Power Loss**: Machine power-on signal from I/O controller goes FALSE
2. **Fault Conditions**: Any servo reports a fault
3. **Communication Failure**: Watchdog timer expires
4. **E-stop Activation**: Emergency stop input is asserted

### Safe Enable Logic

Servos can only be enabled when ALL of the following are TRUE:
- Machine power is on (`ldcn.power-on` = TRUE)
- No faults detected (`ldcn.fault` = FALSE)
- E-stop not active (`ldcn.estop` = FALSE)
- Communication healthy (`ldcn.comms-ok` = TRUE)
- Enable requested (`ldcn.enable-request` = TRUE)

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
  --test-faults     Continuously monitor servos for faults

Examples:
  # Basic diagnostic test
  ./utils/ldcn_diagnostic.py --port /dev/ttyUSB0

  # Test power-on detection
  ./utils/ldcn_diagnostic.py --port /dev/ttyUSB0 --test-power

  # Monitor for faults (checksum errors, current limit, position errors)
  ./utils/ldcn_diagnostic.py --port /dev/ttyUSB0 --test-faults
```

The fault monitoring feature continuously checks all servos and reports:
- **Checksum Errors**: Communication data corruption
- **Current Limit**: Motor drawing excessive current (binding, overload)
- **Position Error**: Servo cannot maintain position (mechanical issue, tuning)

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
