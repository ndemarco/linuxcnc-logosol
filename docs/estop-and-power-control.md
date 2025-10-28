# E-Stop and Power Control Documentation

## Overview

The Logosol LS-2310g2 Supervisor I/O Controller manages safety functions including E-stop monitoring and motor power control. Understanding how these systems work is critical for proper integration with LinuxCNC.

## Critical Concept: Status Byte vs Digital Inputs

**IMPORTANT**: E-stop state is **NOT** represented in the digital inputs. It comes from the **status byte** of the supervisor.

### Wrong Approach (Initial Implementation)
```c
// INCORRECT - E-stop is NOT in digital inputs
bool estop_pressed = !(digital_inputs & 0x01);
```

### Correct Approach (Current Implementation)
```c
// CORRECT - E-stop state comes from power_on flag in status byte
bool estop_ok = supervisor_status.power_on;
```

## Status Byte Structure

The supervisor returns a status byte with the following bits (from LS-2310g2 manual page 20):

### Status Byte (Byte 0)
| Bit | Flag | Meaning |
|-----|------|---------|
| 0 | move_done | Movement command completed |
| 1 | checksum_error | Communication checksum error |
| 2 | current_limit | Current limit reached |
| **3** | **power_on** | **Motor power enabled (E-stop clear)** |
| 4 | position_error | Position error fault |
| 5 | home_in_progress | Homing sequence active |

### Auxiliary Status Byte (Byte 1)
| Bit | Flag | Meaning |
|-----|------|---------|
| 0 | servo_on | Servo amplifier enabled |
| 1 | path_mode | Path mode active |
| 2 | servo_overrun | Servo loop timing overrun |

## Power Control State Machine

### Diagnostic Codes (from LS-2310g2 manual page 20)

The supervisor uses diagnostic codes in Byte1 bits 7-3 to indicate system state:

| Code | Byte1[7:3] | State | Power Enable | Meaning |
|------|------------|-------|--------------|---------|
| 0x01 | 00001 | Initializing | Off | System starting up |
| 0x10 | 10000 | **Emergency Stop** | **Off** | **E-stop button pressed** |
| 0x17 | 10111 | Ready (covers closed) | Off | Waiting for power button |
| 0x1F | 11111 | Running | On | Normal operation |

### Power States

1. **Power Never Enabled** (Initial State)
   - `power_on = false`
   - This is NORMAL during initialization
   - Waiting for operator to press power button
   - NOT an E-stop condition

2. **Power Enabled** (Normal Operation)
   - `power_on = true`
   - E-stop is clear
   - Motors can run
   - LinuxCNC pin `ldcn.estop-ok = TRUE`

3. **E-Stop Activated** (Emergency)
   - `power_on = false` (after being true)
   - E-stop button pressed OR safety circuit opened
   - Motors immediately disabled
   - LinuxCNC pin `ldcn.estop-ok = FALSE`

## Implementation Details

### HAL Pins

```c
ldcn.estop-ok          (bit, OUT)  - TRUE when E-stop is NOT active (power is on)
ldcn.din-raw           (u32, OUT)  - Raw 16-bit digital input word
ldcn.homing-active     (bit, OUT)  - TRUE when any axis is homing
```

### E-Stop Detection Logic

Located in `ldcn.c:update_supervisor()`

```c
/* E-stop state comes from the status byte, not digital inputs
 * When E-stop is active, power_on flag will be false
 * estop_ok should be TRUE when E-stop is NOT active (power_on = true) */
bool prev_estop_ok = *hal_data->estop_ok;
*hal_data->estop_ok = hal_data->supervisor_status.power_on;

/* Print message when E-stop state changes
 * Only show "E-STOP PRESSED" when transitioning from power ON to power OFF
 * (not during initial startup when power was never on) */
if (prev_estop_ok && !hal_data->supervisor_status.power_on) {
    fprintf(stderr, "ldcn: !!! E-STOP PRESSED !!! (power lost)\n");
} else if (!prev_estop_ok && hal_data->supervisor_status.power_on) {
    fprintf(stderr, "ldcn: Power enabled (E-stop clear)\n");
}
```

### Key Design Decision: Initial State

The `estop_ok` pin starts as **FALSE** (line 963 in ldcn.c):

```c
*hal_data->estop_ok = 0;  /* Start as FALSE until power is on */
```

**Rationale**:
- Before power button is pressed, power is genuinely off
- LinuxCNC should know the machine is not ready
- Prevents false "E-STOP PRESSED" messages during initialization
- Only transitions to TRUE when power is actually enabled

## Message Flow During Normal Startup

```
1. ldcn: Auto-detecting baud rate...
2. ldcn: Trying 19200 baud...
3. ldcn: Trying 125000 baud...
4. ldcn: Found devices at 125000 baud
5. ldcn: Sending hard reset to all devices...
6. ldcn: Reset complete (devices at 19200 baud)
7. ldcn: Assigning addresses to 6 devices...
   ... (device addressing) ...
8. ldcn: Upgrading from 19200 to 125000 baud...
9. ldcn: Communication upgraded to 125000 baud
10. ldcn: Configuring supervisor for status reporting...
11. ldcn: Supervisor configured
12. ldcn: Waiting for power button press...
13. ldcn: (Press Ctrl+C to abort)

    [Operator presses power button]

14. ldcn: Power button pressed - power is ON
15. ldcn: Power enabled (E-stop clear)  ← First time power_on becomes true
16. ldcn: Initializing 1 servo drive(s)...
17. ldcn: Axis 0 initialized
18. ldcn: All servo drives initialized
19. ldcn: Initialization complete - entering control loop
20. ldcn: Monitoring E-stop and updating axes at 1kHz
```

## E-Stop Response During Operation

### When E-Stop is Pressed

```
ldcn: !!! E-STOP PRESSED !!! (power lost)
```

**Actions Taken**:
1. `estop_ok` pin set to FALSE
2. All enabled axes receive STOP_MOTOR_OFF command
3. Motors are immediately disabled in hardware
4. LinuxCNC will see `ldcn.estop-ok` go FALSE and can respond accordingly

### When E-Stop is Released

```
ldcn: Power enabled (E-stop clear)
```

**Actions Taken**:
1. `estop_ok` pin set to TRUE
2. LinuxCNC can re-enable motion
3. Individual axis enable pins control whether motors run

## Homing Status

The `ldcn.homing-active` pin reflects the `home_in_progress` status bit (bit 5) from any axis:

```c
/* Update homing-active pin: TRUE if any axis is homing */
bool any_homing = false;
for (int i = 0; i < hal_data->num_axes; i++) {
    axis_data_t *axis = &hal_data->axes[i];
    if (axis->initialized && axis->status.home_in_progress) {
        any_homing = true;
        break;
    }
}
*hal_data->homing_active = any_homing;
```

LinuxCNC can monitor this pin to know when homing operations are in progress.

## Digital Inputs vs Status

### Digital Inputs (`ldcn.din-raw`)
- 16-bit word from supervisor
- General-purpose I/O
- Can be used for limit switches, probe, etc.
- **NOT used for E-stop detection**

### Status Byte (Internal)
- Contains critical system state
- `power_on` flag indicates E-stop state
- `home_in_progress` flag indicates homing status
- Other flags for servo state, errors, etc.

## LinuxCNC Integration

### Recommended HAL Configuration

```hal
# E-stop chain
net estop-ext <= ldcn.estop-ok
net estop-ext => iocontrol.0.emc-enable-in

# Homing status
net homing-active <= ldcn.homing-active
# (Can be used to disable certain operations during homing)

# Axis connections
net x-enable => ldcn.0.enable
net x-pos-cmd => ldcn.0.position-cmd
net x-pos-fb <= ldcn.0.position-fb
# ... etc for other axes
```

### E-Stop Behavior

- When `ldcn.estop-ok` goes FALSE:
  - LinuxCNC sees external E-stop condition
  - Motion controller stops all motion
  - System goes into E-stop state

- When `ldcn.estop-ok` goes TRUE:
  - LinuxCNC allows E-stop reset
  - Operator must still manually reset E-stop in LinuxCNC GUI
  - Motion can resume after reset

## Safety Considerations

1. **Hardware E-Stop Always Takes Priority**
   - The LS-2310g2 supervisor controls motor power with hardware relays
   - Even if software fails, motors will be disabled
   - This is a safety-critical design

2. **LinuxCNC Should Monitor estop-ok**
   - Connect `ldcn.estop-ok` to `iocontrol.0.emc-enable-in`
   - This provides proper E-stop integration
   - LinuxCNC will track external E-stop state

3. **Never Override E-Stop in Software**
   - Do not attempt to keep motors running when `power_on = false`
   - The driver automatically sends STOP_MOTOR_OFF commands
   - This is intentional safety behavior

## Troubleshooting

### "E-STOP PRESSED" showing during startup
**Symptom**: Message appears before power button is pressed
**Cause**: Old implementation that didn't distinguish initial state
**Fix**: Current implementation (fixed) - estop_ok starts FALSE and only messages on state changes

### E-Stop not detected when pressed
**Symptom**: Motors continue running after E-stop press
**Cause**: Wrong detection method (checking digital inputs)
**Fix**: Use `supervisor_status.power_on` flag from status byte

### E-Stop not releasing
**Symptom**: `estop_ok` stays FALSE after releasing E-stop button
**Cause**: Power button needs to be pressed again after E-stop
**Fix**: Normal behavior - operator must restart machine

## References

- **LS-2310g2 Manual**: `docs/LS-2310g2-Supervisor-IO-Controller.pdf`
  - Page 20: Diagnostic codes and status byte meanings
  - Safety circuit diagrams and wiring examples

- **LDCN Protocol**: `ldcn_protocol.h`
  - Status byte bit definitions
  - `ldcn_drive_status_t` structure

- **Implementation**: `ldcn.c`
  - `update_supervisor()` function (lines 630-690)
  - E-stop detection and axis disable logic

## Revision History

- **2025-10-28**: Initial documentation created
  - Documented correct E-stop detection method (status byte, not digital inputs)
  - Explained power_on flag meaning and state transitions
  - Added homing status pin documentation
  - Clarified difference between "power not on" vs "E-stop pressed"
