# Nicky 5-Axis Mill LinuxCNC Configuration

## Overview

This LinuxCNC configuration controls a 5-axis XYZAC mill using Logosol LDCN servo drives. The configuration is based on production-tested parameters from the original Mctl_Logosol1.ini file.

## Machine Specifications

### Axes Configuration
- **Coordinates**: XYZAC (5 axes)
- **Kinematics**: trivkins (simple Cartesian + 2 rotary)
- **Linear Axes**: X, Y, Z (millimeters)
- **Rotary Axes**: A (rotates about X), C (rotates about Z)

### Travel Limits

| Axis | Type | Min Limit | Max Limit | Units |
|------|------|-----------|-----------|-------|
| X | Linear | -2 mm | 310 mm | mm |
| Y | Linear | -5 mm | 185 mm | mm |
| Z | Linear | -120 mm | 2 mm | mm |
| A | Rotary | -9999° | 9999° | degrees |
| C | Rotary | -9999° | 9999° | degrees |

### Velocity Limits

| Axis | Max Velocity | Default Velocity |
|------|--------------|------------------|
| X | 500 mm/s | 25 mm/s |
| Y | 100 mm/s | 25 mm/s |
| Z | 100 mm/s | 25 mm/s |
| A | 100 °/s | — |
| C | 5 °/s | — |

### Acceleration Limits
- All axes: 100 mm/s² or 100 °/s²

### Spindle Specifications
- **Speed Range**: 5,000 - 60,000 RPM
- **Control**: 0-10V analog output (LS-2310g2)
- **Feedback**: 0-10V analog input (LS-2310g2)
- **Direction**: Forward only
- **Air Bearing**: Required whenever spindle is running

### Digital I/O
- **Digital Inputs**: 16 available (tool sensor, air pressure, etc.)
- **Digital Outputs**: 4 solenoids (air bearing, tool change, air blast, coolant)

## Critical Configuration Details

### LDCN Address Mapping

**IMPORTANT**: The LDCN hardware addresses DO NOT match the logical axis names!

| Joint | Axis Name | LDCN Address | HAL Component |
|-------|-----------|--------------|---------------|
| 0 | Y | 1 | ldcn.0 |
| 1 | X | 2 | ldcn.1 |
| 2 | Z | 3 | ldcn.2 |
| 3 | A | 4 | ldcn.3 |
| 4 | C | 5 | ldcn.4 |

This mapping is implemented in the HAL file and reflects the actual hardware addresses from the original Mctl_Logosol1.ini configuration.

### Servo Update Rate

- **Servo Period**: 1.024 ms (1,024,000 nanoseconds)
- **Servo Rate Divisor**: 20
- **Update Rate**: 976 Hz
- **Tick Time**: 51.2 µs × 20 = 1.024 ms

This non-realtime-friendly configuration allows LinuxCNC to run without a realtime kernel.

### Scale Factors

| Axis | Scale | Calculation |
|------|-------|-------------|
| X | 2000 cnt/mm | 10000 cnt/rev ÷ 5 mm pitch |
| Y | 2000 cnt/mm | 10000 cnt/rev ÷ 5 mm pitch |
| Z | 4000 cnt/mm | 10000 cnt/rev ÷ 2.5 mm pitch |
| A | 2805.56 cnt/° | (10000 cnt/rev × 101 gear) ÷ 360° |
| C | 2805.56 cnt/° | (10000 cnt/rev × 101 gear) ÷ 360° |

### PID Gains (Production-Tested)

**Linear Axes (X, Y, Z):**
- KP = 10
- KD = 1000
- KI = 20

**Rotary Axes (A, C):**
- KP = 110
- KD = 5000
- KI = 10

These values were taken from the production Mctl_Logosol1.ini and have been proven stable in operation.

### Homing Configuration

**Homing Sequence**: Z → Y → X → A → C

The Z axis homes first for safety (moves up and away from workpiece).

**Homing Parameters**:

| Axis | Search Vel | Latch Vel | Direction | Home Distance | Use Index |
|------|------------|-----------|-----------|---------------|-----------|
| X | 20 mm/s | 5 mm/s | Positive | 5 mm | Yes |
| Y | 20 mm/s | 5 mm/s | Positive | 5 mm | Yes |
| Z | -20 mm/s | -5 mm/s | Negative | -5 mm | Yes |
| A | 1 °/s | 0.5 °/s | Positive | 0.1° | Yes |
| C | 3 °/s | 0.1 °/s | Positive | 0.285° | No |

## File Structure

```
config/nicky_5axis_mill/
├── logosol.ini          # Main INI configuration
├── logosol.hal          # Main HAL connections
├── custom.hal           # User customizations
├── postgui.hal          # GUI-specific connections
├── shutdown.hal         # Shutdown procedures
├── tool.tbl             # Tool table
├── logosol.var          # G-code variables (auto-generated)
├── position.txt         # Position save file (auto-generated)
└── README.md            # This file
```

## Installation

### Prerequisites

1. LinuxCNC 2.8 or later installed
2. LDCN driver compiled and installed (see src/README.md)
3. Serial connection to LS-2310g2 controller (typically /dev/ttyUSB0)
4. User has permission to access serial port:
   ```bash
   sudo usermod -aG dialout $USER
   # Log out and back in for group membership to take effect
   ```

### Testing the Configuration

Before running the full LinuxCNC GUI, test the HAL connections:

```bash
cd config/nicky_5axis_mill
halrun -I logosol.hal
```

Expected startup sequence:
1. Auto-detecting baud rate
2. Hard reset to all devices
3. Assigning addresses to 6 devices
4. Upgrading to 125000 baud
5. Configuring supervisor
6. Waiting for power button press (press power on machine)
7. Initializing servo drives
8. Entering control loop at 1kHz

### Running LinuxCNC

```bash
linuxcnc config/nicky_5axis_mill/logosol.ini
```

## E-Stop and Power Control

### Important Behavior

The machine uses a hardware E-stop system managed by the LS-2310g2 Supervisor:

1. **Power Not Yet Enabled** (Initial State)
   - After startup, power button must be pressed
   - `ldcn.estop-ok` = FALSE
   - This is normal, not an emergency

2. **Power Enabled** (Normal Operation)
   - Power button has been pressed
   - `ldcn.estop-ok` = TRUE
   - Motors can run

3. **E-Stop Activated** (Emergency)
   - E-stop button pressed OR safety circuit opened
   - `ldcn.estop-ok` = FALSE
   - All motors immediately disabled

### E-Stop Recovery

To recover from an E-stop condition:

1. Release physical E-stop button
2. Press power button on machine
3. Reset E-stop in LinuxCNC GUI (F1 or E-stop button)
4. Machine is ready to operate

See `docs/estop-and-power-control.md` for detailed technical documentation.

## Tuning the PID Gains

The default PID gains are production-tested values. Only adjust if experiencing problems.

### Symptoms and Adjustments

**Motor oscillates or chatters:**
- Reduce KP (proportional gain)
- Increase KD (derivative gain)

**Poor following accuracy:**
- Increase KP (proportional gain)
- Reduce KI (integral gain) if overshooting

**Long-term position drift:**
- Increase KI (integral gain)
- Check for mechanical binding

### Tuning Procedure

1. Start with current values
2. Make small changes (10-20% at a time)
3. Test with gentle moves first
4. Monitor following error in HAL meter:
   ```bash
   halmeter pin ldcn.X.following-error
   ```
5. Ensure error stays within FERROR limits (5mm or 5°)

### HAL Parameter Setting

To change gains during operation:

```bash
halcmd setp ldcn.0.kp 12    # Increase Y axis proportional gain
halcmd setp ldcn.1.kd 1100  # Increase X axis derivative gain
```

To make changes permanent, edit `logosol.hal` lines 35-37 (for example).

## Customization

### Adding Digital I/O

The supervisor provides 16 digital inputs via `ldcn.din-raw`. To use them:

1. Edit `custom.hal`
2. Add connections, for example:
   ```hal
   # Probe input on bit 0
   net probe-in ldcn.din-raw => motion.probe-input
   ```

### Adding Jog Pendant

The reference configuration (Mctl_Logosol1.ini) shows MPG (Manual Pulse Generator) settings. To implement:

1. Connect MPG hardware
2. Create HAL component for MPG encoder
3. Connect to axis jog controls in `custom.hal`

See LinuxCNC documentation for MPG configuration examples.

### GUI Customization

The configuration uses the standard AXIS GUI. To customize:

1. Edit `postgui.hal` for GUI-specific connections
2. Or switch to a different GUI by changing DISPLAY in `logosol.ini`

Available GUIs: axis, gmoccapy, qtdragon, touchy

## Troubleshooting

### "Can't open serial port /dev/ttyUSB0"

**Cause**: Permission denied or wrong device

**Solutions**:
1. Check device exists: `ls -l /dev/ttyUSB*`
2. Check permissions: User must be in `dialout` group
3. Try different device: `/dev/ttyACM0`, `/dev/ttyS0`, etc.

### "Following Error Exceeded"

**Cause**: PID gains need tuning or mechanical binding

**Solutions**:
1. Check for mechanical problems first
2. Reduce feed rates and accelerations
3. Tune PID gains (see Tuning section above)
4. Increase FERROR limits in INI (only as last resort)

### "E-Stop Active" on Startup

**Cause**: Power button not yet pressed (normal behavior)

**Solution**: Press power button on machine. The driver waits for power before initializing servo drives.

### Motors Don't Move

**Checks**:
1. E-stop released? Check `ldcn.estop-ok` is TRUE
2. LinuxCNC enabled? Press F2 or click Machine > Enable
3. Axis enabled? Check `ldcn.X.amp-enabled` pins
4. Following error? Check error lights and messages

### Communication Errors

**Symptoms**: "Checksum error" messages

**Solutions**:
1. Check serial cable quality and length
2. Verify baud rate (should auto-detect 125000)
3. Check for electrical noise near cable
4. Try different USB port

## Serial Port Configuration

The LDCN driver automatically:
1. Auto-detects baud rate (tries 19200, then 125000)
2. Sends hard reset to all devices
3. Assigns addresses to devices in order
4. Upgrades communication to 125000 baud
5. Configures supervisor for status reporting

No manual serial configuration is needed.

## Safety Notes

1. **Hardware E-Stop Always Takes Priority**
   - The supervisor controls motor power with hardware relays
   - Even if software fails, motors will be disabled

2. **Homing Sequence is Critical**
   - Z axis MUST home first to avoid collisions
   - Do not modify HOME_SEQUENCE without understanding implications

3. **Soft Limits are Active**
   - Machine will stop at configured soft limits
   - Limits are based on production configuration
   - Adjust only if machine physical limits have changed

4. **Following Error Detection**
   - If position error exceeds 5mm/5°, motion stops
   - This prevents runaway conditions
   - Check mechanical binding if errors occur

## References

- **LDCN Driver Documentation**: `../../src/README.md`
- **E-Stop Technical Details**: `../../docs/estop-and-power-control.md`
- **Spindle and I/O Control**: `../../docs/spindle-and-io.md`
- **LDCN Protocol**: `../../TECHNICAL.md`
- **Original Configuration**: `../../reference/Mctl_Logosol1.ini`
- **LinuxCNC Documentation**: http://linuxcnc.org/docs/

## Revision History

- **2025-10-28**: Initial configuration created
  - Based on production parameters from Mctl_Logosol1.ini
  - Servo rate divisor = 20 (1.024ms update rate)
  - Correct LDCN address mapping implemented
  - Production-tested PID gains applied
