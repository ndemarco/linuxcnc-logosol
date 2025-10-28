# Spindle and I/O Control Documentation

## Overview

The Logosol sawmill uses the LS-2310g2 Supervisor for spindle control and pneumatic solenoid management. This document describes the hardware configuration and LinuxCNC integration requirements.

## Spindle Control

### Hardware Specifications

- **Control**: LS-2310g2 Analog Output (0 or 1 - to be confirmed)
- **Feedback**: LS-2310g2 Analog Input (0 or 1 - to be confirmed)
- **Voltage Range**: 0-10 VDC
- **Speed Range**: 5,000 - 60,000 RPM
- **Speed Scaling**: Linear (0V = 5000 RPM, 10V = 60,000 RPM)
- **Direction**: Forward only (no reverse)
- **Additional Signals**: Enable, Fault

### Voltage to RPM Conversion

```
RPM = (Voltage × 5500) + 5000
Voltage = (RPM - 5000) / 5500
```

**Examples**:
- 0V → 5,000 RPM (minimum)
- 5V → 32,500 RPM (mid-range)
- 10V → 60,000 RPM (maximum)

### LinuxCNC Integration

The spindle is controlled via the standard LinuxCNC spindle interface:

**HAL Pins**:
```hal
spindle.0.speed-out     (float, OUT) - Commanded speed in RPM
spindle.0.speed-in      (float, IN)  - Actual speed in RPM
spindle.0.on            (bit, OUT)   - TRUE when spindle should run
spindle.0.forward       (bit, OUT)   - TRUE for forward rotation
spindle.0.at-speed      (bit, IN)    - TRUE when spindle is at commanded speed
```

**Scaling Components**:

Two `scale` components convert between RPM and voltage:

1. **spindle-speed-scale** (RPM → Voltage for command)
   - Gain: 0.000181818 (10V / 55000 RPM range)
   - Offset: -0.909091 (for 5000 RPM minimum)

2. **spindle-fb-scale** (Voltage → RPM for feedback)
   - Gain: 5500.0 (55000 RPM / 10V)
   - Offset: 5000.0 (5000 RPM minimum)

### G-Code Commands

- `M3 S10000` - Start spindle forward at 10,000 RPM
- `M5` - Stop spindle
- `S20000` - Change speed to 20,000 RPM (while running)

## Digital Outputs (Air Solenoids)

The LS-2310g2 provides digital outputs for pneumatic control. Four solenoids are used:

### Output Assignments

| Output | Function | Control | Critical |
|--------|----------|---------|----------|
| 0 | Spindle air bearing | Automatic | **YES** |
| 1 | Tool change release | LinuxCNC | No |
| 2 | Air blast (mist) | M7 | No |
| 3 | Coolant blast (flood) | M8 | No |

### Output 0: Spindle Air Bearing (CRITICAL)

**Purpose**: Provides compressed air to the spindle's air bearings.

**Critical Safety Requirements**:
- **MUST** be enabled whenever the spindle is running
- **MUST** remain enabled during spindle spin-down
- Failure to maintain air pressure will **destroy the spindle**

**Implementation**:

An AND gate (`spindle-air-interlock`) ensures the air bearing is on when:
1. Spindle is commanded on (`spindle.0.on`), OR
2. Spindle is at speed (`spindle.0.at-speed`)

This keeps air flowing during startup, operation, and spin-down.

```hal
loadrt and2 names=spindle-air-interlock
net spindle-on => spindle-air-interlock.in0
net spindle-at-speed => spindle-air-interlock.in1
net spindle-air-bearing spindle-air-interlock.out => ldcn.dout-0
```

**Safety Note**: Consider adding a time delay to keep air on for several seconds after spindle stops to ensure complete spin-down.

### Output 1: Tool Change Release

**Purpose**: Releases the tool holder during automatic tool changes.

**Control**: Managed by LinuxCNC's tool change sequence

**HAL Connections**:
```hal
net tool-change iocontrol.0.tool-change => ldcn.dout-1
net tool-changed iocontrol.0.tool-changed
```

**Sequence**:
1. LinuxCNC sets `tool-change` TRUE
2. Tool holder releases (spring-loaded)
3. Operator or automatic changer swaps tool
4. Operator confirms (or sensor detects)
5. LinuxCNC sets `tool-change` FALSE
6. Tool holder clamps

### Output 2: Air Blast (Mist Coolant)

**Purpose**: Blows compressed air to clear chips

**Control**: M7 G-code command

**HAL Connection**:
```hal
net air-blast iocontrol.0.coolant-mist => ldcn.dout-2
```

**G-Code**:
- `M7` - Turn on air blast
- `M9` - Turn off all coolant/air

### Output 3: Coolant Blast (Flood Coolant)

**Purpose**: Sprays cutting fluid or coolant

**Control**: M8 G-code command

**HAL Connection**:
```hal
net coolant-blast iocontrol.0.coolant-flood => ldcn.dout-3
```

**G-Code**:
- `M8` - Turn on coolant
- `M9` - Turn off all coolant/air

## Digital Inputs

The LS-2310g2 provides 16 digital inputs. Currently mapped:

### Input 6: Tool Length Sensor

**Purpose**: Automatically measures tool length for tool offset calculation

**Type**: Normally-open contact (closes when tool touches sensor)

**HAL Connection**:
```hal
net tool-length-sensor ldcn.din-6 => motion.probe-input
```

**Usage**:
1. Install tool
2. G-code command `G38.2 Z-50 F10` (probe downward)
3. Machine stops when tool touches sensor
4. Tool offset calculated from current position
5. Stored in tool table

**Safety**: Always set maximum probe distance to prevent crashes

### Input 7: Air Pressure Sufficient (TBD)

**Purpose**: Confirms compressed air system has adequate pressure

**Note**: Input number to be confirmed (using 7 as placeholder)

**Recommended Implementation**:

```hal
# Interlock machine enable with air pressure
loadrt and2 names=air-pressure-interlock
net air-pressure-ok ldcn.din-7 => air-pressure-interlock.in0
net estop-ok ldcn.estop-ok => air-pressure-interlock.in1
net machine-enable-ok air-pressure-interlock.out => iocontrol.0.emc-enable-in
```

This prevents machine operation when air pressure is low, which is critical for:
- Spindle air bearing operation
- Tool clamping reliability
- Solenoid operation

## Driver Implementation Requirements

### Currently Missing Features

The LDCN driver currently implements:
- ✅ Servo axis control (position, velocity, PID)
- ✅ Digital inputs (16-bit word `ldcn.din-raw`)
- ✅ E-stop monitoring
- ✅ Homing status

The following features need to be added:

### 1. Analog Outputs

**Required HAL Pins**:
```
ldcn.aout-0    (float, IN) - Analog output 0 (0-10V)
ldcn.aout-1    (float, IN) - Analog output 1 (0-10V)
```

**Implementation Notes**:
- Voltage range: 0.0 to 10.0 (floating point)
- Update rate: Same as servo loop (1 kHz)
- Resolution: 12-bit or better (0.0024V steps)
- Use LDCN protocol command for analog output

**LDCN Protocol**:
Check LS-2310g2 manual for analog output command format.

### 2. Analog Inputs

**Required HAL Pins**:
```
ldcn.ain-0     (float, OUT) - Analog input 0 (0-10V)
ldcn.ain-1     (float, OUT) - Analog input 1 (0-10V)
```

**Implementation Notes**:
- Voltage range: 0.0 to 10.0 (floating point)
- Update rate: Same as servo loop (1 kHz)
- Resolution: 12-bit or better (0.0024V steps)
- Read from supervisor status or separate query

**LDCN Protocol**:
Analog inputs may be included in supervisor status response.

### 3. Digital Outputs

**Required HAL Pins**:
```
ldcn.dout-raw  (u32, IN)  - Raw 16-bit output word
ldcn.dout-0    (bit, IN)  - Digital output 0
ldcn.dout-1    (bit, IN)  - Digital output 1
...
ldcn.dout-15   (bit, IN)  - Digital output 15
```

**Implementation Notes**:
- Provide both raw word and individual bit pins
- Update rate: Same as servo loop (1 kHz)
- Use LDCN protocol command for digital output
- Supervisor should latch output state

**LDCN Protocol**:
Check LS-2310g2 manual for digital output command format.

### 4. Individual Digital Input Pins

**Required HAL Pins**:
```
ldcn.din-raw   (u32, OUT) - Raw 16-bit input word (already exists)
ldcn.din-0     (bit, OUT) - Digital input 0
ldcn.din-1     (bit, OUT) - Digital input 1
...
ldcn.din-15    (bit, OUT) - Digital input 15
```

**Implementation Notes**:
- Extract individual bits from existing `din-raw` word
- Makes HAL connections cleaner and more readable
- No protocol changes needed

## Testing Procedure

### Phase 1: Digital I/O (No Spindle)

1. **Implement digital outputs**
   - Test by toggling outputs manually with `halcmd`
   - Observe solenoid operation
   - Verify no false triggers

2. **Implement individual digital input pins**
   - Test tool length sensor
   - Test air pressure switch
   - Verify correct bit mapping

### Phase 2: Analog I/O (Spindle Speed)

1. **Implement analog outputs**
   - Test with fixed voltages (0V, 5V, 10V)
   - Measure spindle speed with tachometer
   - Verify voltage-to-RPM scaling

2. **Implement analog inputs**
   - Compare analog input reading to tachometer
   - Verify feedback scaling is correct
   - Test at various speeds (min, mid, max)

### Phase 3: Integrated Testing

1. **Spindle control from LinuxCNC**
   - Test M3 S10000 (start at 10,000 RPM)
   - Test S20000 (change speed while running)
   - Test M5 (stop)

2. **Air bearing interlock**
   - Verify air turns on with spindle
   - Verify air stays on during spin-down
   - Test emergency stop behavior

3. **Tool change sequence**
   - Test manual tool change with M6
   - Test tool length probe with G38.2
   - Verify tool offsets are correct

## Safety Considerations

### Spindle Air Bearing

**CRITICAL**: The spindle air bearing is a safety-critical system.

**Requirements**:
1. Air MUST be on before spindle starts
2. Air MUST remain on during operation
3. Air MUST remain on during spin-down
4. Loss of air pressure must trigger E-stop

**Recommended Additions**:
- Add time delay to keep air on for 10-15 seconds after spindle stops
- Monitor air pressure via digital input
- Interlock spindle enable with air pressure
- Add alarm if air fails while spindle is running

### Air Pressure Monitoring

Connect air pressure switch to E-stop chain:

```hal
# Require both E-stop clear AND air pressure OK
loadrt and2 names=safety-interlock
net estop-ok ldcn.estop-ok => safety-interlock.in0
net air-pressure-ok ldcn.din-7 => safety-interlock.in1
net machine-enable safety-interlock.out => iocontrol.0.emc-enable-in
```

This prevents:
- Spindle operation without air bearing
- Tool changes without clamping pressure
- Operation with low air pressure

## G-Code Examples

### Spindle Operation

```gcode
(Start spindle at 18,000 RPM)
M3 S18000
G4 P2          (Wait 2 seconds for spindle to reach speed)

(Your cutting operations here)

M5             (Stop spindle)
```

### Tool Change with Automatic Measurement

```gcode
(Load tool 1 and measure length)
M6 T1          (Change to tool 1)
G43 H1         (Apply tool 1 offset)
G38.2 Z-50 F10 (Probe for tool length)
G10 L1 P1 Z[#5063] (Store measured Z in tool 1 offset)
G0 Z10         (Retract)
```

### Coolant/Air Control

```gcode
M3 S20000      (Start spindle)
M8             (Coolant on)
M7             (Air blast on)

(Cutting operations)

M9             (All coolant/air off)
M5             (Spindle off)
```

## References

- **LS-2310g2 Manual**: `docs/LS-2310g2-Supervisor-IO-Controller.pdf`
  - Analog I/O specifications
  - Digital I/O specifications
  - Command protocol for outputs

- **LinuxCNC HAL Reference**: http://linuxcnc.org/docs/html/hal/
  - Spindle control
  - I/O control
  - Tool changing

- **LDCN Driver**: `src/ldcn.c`
  - Current implementation
  - Locations for I/O additions

## Revision History

- **2025-10-28**: Initial documentation created
  - Spindle control specifications (0-10V, 5000-60000 RPM)
  - Digital output assignments (4 solenoids)
  - Digital input assignments (tool sensor, air pressure)
  - Driver implementation requirements
  - Safety considerations for air bearing
