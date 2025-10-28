# LinuxCNC Logosol LDCN Driver - Technical Documentation

## Table of Contents

1. [Protocol Overview](#protocol-overview)
2. [Architecture](#architecture)
3. [Configuration](#configuration)
4. [PID Tuning](#pid-tuning)
5. [Troubleshooting](#troubleshooting)
6. [Performance](#performance)

## Protocol Overview

### Communication Protocol

The LDCN protocol is a master/slave RS-485 protocol:
- **Baud Rates**: 19200 to 1,250,000 bps
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Master**: LinuxCNC host
- **Slaves**: Servo drives (up to 31)

### Command Packet Structure

```
[Header][Address][Command][Data 0-15][Checksum]
  0xAA    1 byte   1 byte   variable   1 byte
```

**Command Byte**: `[Data Length (4 bits)][Command Code (4 bits)]`
**Checksum**: 8-bit sum of Address + Command + Data

### Status Packet Structure

```
[Status][Data 0-23][Checksum]
 1 byte  variable   1 byte
```

**Checksum**: 8-bit sum of Status + Data

### Key Commands

| Cmd | Code | Description | Data Bytes |
|-----|------|-------------|------------|
| Hard Reset | 0xF | Reset all drives | 0 |
| Set Address | 0x1 | Assign addresses | 2 |
| Define Status | 0x2 | Configure status | 2 |
| Load Trajectory | 0x4 | Set pos/vel/accel | 1-15 |
| Set Gain | 0x6 | Configure PID | 14 |
| Stop Motor | 0x7 | Stop with modes | 1 |

### Data Encoding

**Position** (32-bit signed): Direct encoder counts, LSB first

**Velocity** (32-bit unsigned): `counts/tick × 65536`
```
velocity_raw = (counts/sec) × (tick_time) × 65536
tick_time = 51.2µs × servo_rate_divisor
```

**Acceleration** (32-bit unsigned): `counts/tick² × 65536`
```
accel_raw = (counts/sec²) × (tick_time)² × 65536
```

### Servo Update Rate

```
Tick Time = 51.2 µs × SR
Update Rate = 19531 Hz / SR

SR = 1:  51.2 µs (19,531 Hz) - realtime required
SR = 20: 1.024 ms (976 Hz)   - non-realtime friendly ✓
SR = 50: 2.56 ms (391 Hz)    - very conservative
```

**Recommended for non-realtime**: SR = 20 (~1ms)

## Architecture

### Component Structure

```
LinuxCNC Motion Controller
         ↓ HAL pins
    ldcn HAL Component
         ↓ 
    ldcn_protocol (commands)
         ↓
    ldcn_serial (RS-485)
         ↓
    LS-832RL USB Adapter
         ↓
    LS-231SE Servo Drives
```

### File Structure

```
src/
├── ldcn.c            # Main HAL component
├── ldcn_protocol.c   # Protocol implementation
├── ldcn_protocol.h   # Protocol definitions
├── ldcn_serial.c     # Serial communication
├── ldcn_serial.h     # Serial interface
└── Makefile          # Build system
```

### LDCN Initialization Sequence

The driver performs a standardized initialization sequence that follows the LDCN protocol requirements:

**Step 1: Open Communications (19200 baud)**
- Opens serial port at 19200 baud (LDCN protocol default)
- All LDCN devices power up at 19200 baud by default
- Required for initial communication before baud upgrade

**Step 2: Reset All Devices**
- Sends `HARD_RESET` command to broadcast address (0xFF)
- Resets all drives to known state
- Clears any error conditions
- 100ms delay for devices to complete reset

**Step 3: Initialize Each Drive**
- Sets individual addresses (1, 2, 3, ...)
- Configures status reporting format
- Sets PID gains (Kp, Kd, Ki)
- Loads initial trajectory parameters
- Enables servo amplifiers

**Step 4: Upgrade Baud Rate (Optional)**
- If configured baud ≠ 19200, upgrades to target speed
- Sends `SET_BAUD` command to all drives
- Changes host serial port baud rate
- Verifies communication at new speed
- Supported rates: 57600, 115200, 125000, 312500, 625000, 1250000

**Example Log Output:**
```
ldcn: Opening /dev/ttyUSB0 at 19200 baud (LDCN default)
ldcn: Resetting all LDCN devices...
ldcn: Initializing 3 drives at 19200 baud...
ldcn: Initializing axis 0 (addr 0x01)
ldcn: Axis 0 initialized successfully
ldcn: Initializing axis 1 (addr 0x02)
ldcn: Axis 1 initialized successfully
ldcn: Initializing axis 2 (addr 0x03)
ldcn: Axis 2 initialized successfully
ldcn: Upgrading communication speed to 125000 baud...
ldcn: Verifying communication at 125000 baud...
ldcn: Successfully upgraded to 125000 baud
ldcn: Entering main loop
```

**Why This Sequence?**
- LDCN protocol requires 19200 baud for initial communication
- Reset ensures all drives start in known state
- Individual addressing allows multi-drop RS-485 network
- Baud upgrade improves update rates and reduces latency

## Configuration

### Basic Setup

1. **Connect Hardware**
```
PC → USB → LS-832RL → RJ45 → Drive 1 → Drive 2 → ... → Drive N
```

2. **Load Driver**
```hal
loadusr -W ldcn port=/dev/ttyUSB0 baud=115200 axes=3
```

3. **Configure Axes**
```hal
setp ldcn.0.scale 400.0
setp ldcn.0.kp 50
setp ldcn.0.kd 8000
setp ldcn.0.ki 50
```

4. **Connect to Joints**
```hal
net x-enable joint.0.amp-enable-out => ldcn.0.enable
net x-pos-cmd joint.0.motor-pos-cmd => ldcn.0.position-cmd
net x-pos-fb ldcn.0.position-fb => joint.0.motor-pos-fb
```

### Scale Calculation

```
SCALE = encoder_counts_per_rev / pitch_per_rev

Examples:
- 2000 CPR, 5mm pitch:  2000/5 = 400 counts/mm
- 2000 CPR, 0.2" pitch: 2000/0.2 = 10000 counts/inch
- 4000 CPR, 10mm pitch: 4000/10 = 400 counts/mm
```

### Baud Rate Selection

| Baud | Use Case |
|------|----------|
| 19200 | Initial testing |
| 57600 | 1-2 axes |
| 115200 | 3-4 axes (recommended) |
| 625000 | Many axes, short cables |
| 1250000 | High performance, <30m |

## PID Tuning

### PID Parameters

- **KP (Position Gain)**: Determines stiffness
  - Too low: Sluggish, large following error
  - Too high: Oscillation
  - Start: 50

- **KD (Velocity Gain)**: Provides damping
  - Too low: Oscillation
  - Too high: Sluggish
  - Start: 8000

- **KI (Integral Gain)**: Eliminates steady-state error
  - Too low: DC error remains
  - Too high: Instability
  - Start: 50 (or 0)

### Tuning Procedure

1. **Start Conservative**
```hal
setp ldcn.N.kp 50
setp ldcn.N.kd 8000
setp ldcn.N.ki 0  # Disable initially
```

2. **Test Response**
- Jog slowly
- Check for smooth motion
- Monitor following error

3. **Increase KP**
- Double if too soft
- Halve if oscillating
- Target: Crisp without overshoot

4. **Adjust KD**
- Increase if oscillating
- Reduce if too damped

5. **Add KI**
- Start with 10-50
- Increase slowly
- Watch for instability

6. **Verify**
- Run test patterns
- Check following error
- Ensure no oscillation

## Troubleshooting

### Drive Won't Initialize

**Checks:**
1. Verify USB: `ls -l /dev/ttyUSB*`
2. Check permissions: `groups`
3. Test different baud rate
4. Check cables
5. Verify drive power

**Solution:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Position Drift

**Causes:** Insufficient KP, encoder noise, ground loops

**Solutions:**
1. Increase KP gain
2. Add/increase KI gain
3. Check encoder cable shielding
4. Verify grounds

### Oscillation

**Causes:** KP too high, KD too low, mechanical resonance

**Solutions:**
1. Reduce KP gain
2. Increase KD gain
3. Check for mechanical play
4. Adjust acceleration limits

### Following Error

**Causes:** Gains too low, acceleration too high, binding

**Solutions:**
1. Increase KP gain
2. Reduce acceleration
3. Check for binding
4. Verify enable signal

### Communication Errors

**Causes:** Cable too long, EMI, baud too high

**Solutions:**
1. Reduce baud rate
2. Use shielded cable
3. Keep away from power lines
4. Add ferrite beads
5. Increase servo_rate

## Performance

### Timing Characteristics

With servo_rate=20 (~1ms):
- **Command to Drive**: 1-2ms
- **Drive Processing**: <0.1ms
- **Status Return**: 1-2ms
- **Total Round-Trip**: 3-5ms

**Jitter** (Non-RT Linux):
- Typical: 0.1-1ms
- Worst: 5-10ms
- Path buffer: Absorbs 1.3s

### Throughput

Bytes per axis per update: ~20-25 bytes

At 115200 baud: ~460 updates/sec max

Practical: 3-4 axes at 1ms with 115200 baud

### CPU Usage

Typical (3 axes, 1ms update):
- Intel i5: 0.5-1%
- RPi 4: 2-3%
- Older: 5-10%

## Advanced Features

### Path Mode

Enable buffered trajectory mode:
```bash
loadusr -W ldcn path_mode=1 path_buffer_freq=100
```

Provides 256-point buffer (~1.3 seconds) for smoother non-RT motion.

### Multiple Machines

Each axis can have different parameters:
```hal
# Fast X/Y axes
setp ldcn.0.scale 400.0
setp ldcn.1.scale 400.0

# Slower Z axis, different pitch
setp ldcn.2.scale 800.0
setp ldcn.2.kp 80
setp ldcn.2.kd 10000
```

## References

- Logosol LS-231SE Manual
- LinuxCNC HAL Documentation
- FTDI FT232R Datasheet

## Support

- GitHub: https://github.com/ndemarco/linuxcnc-logosol
- Forum: https://forum.linuxcnc.org
