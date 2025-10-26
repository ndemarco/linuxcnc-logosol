# Safety Guide for LDCN HAL Component

## Overview

This document provides critical safety information for operating CNC machines controlled by the LDCN HAL component. **READ THIS ENTIRE DOCUMENT BEFORE OPERATING YOUR MACHINE.**

## DANGER: CNC Machine Hazards

CNC machines with servo motors can cause:
- Severe injury or death from moving parts
- Crushing hazards from tool head or workpiece movement
- Electrical shock from high-voltage components
- Fire from electrical faults or improper operation

**NEVER operate a CNC machine without:**
- Proper training and understanding of machine operation
- Functional emergency stop system
- Machine guards and safety interlocks
- Clear understanding of all safety features

## Safety Features

### 1. Emergency Stop System

#### Hardware E-stop Requirements

**CRITICAL**: The LDCN HAL component's `ldcn.estop` pin is a software interface. It MUST be part of a complete emergency stop system that includes:

1. **Physical E-stop Buttons**
   - Red mushroom-head buttons at all operator positions
   - Hardwired to interrupt power to servo amplifiers
   - Latching type (requires reset to restart)

2. **Safety Relay/Contactor**
   - Industrial-grade safety relay controlling servo amplifier power
   - Rated for your system voltage and current
   - Meets safety standards (e.g., EN 418, ISO 13850)

3. **Software Integration**
   - Connect `ldcn.estop` to LinuxCNC's E-stop chain
   - E-stop button state monitored by HAL
   - Provides defense-in-depth with hardware cutoff

#### E-stop HAL Configuration

```hal
# Connect LDCN E-stop to LinuxCNC E-stop chain
net estop-out iocontrol.0.user-enable-out => ldcn.estop

# Monitor fault conditions and trigger E-stop
net ldcn-fault ldcn.fault => iocontrol.0.emc-enable-in
```

#### E-stop Testing

Test emergency stop BEFORE each operating session:

1. Start LinuxCNC and enable machine
2. Press each E-stop button individually
3. Verify all servos disable immediately
4. Verify tool head stops moving
5. Verify system requires reset before re-enabling

**If E-stop does not work properly, DO NOT OPERATE THE MACHINE.**

### 2. Fault Detection

The LDCN component monitors three critical fault conditions:

#### Checksum Errors
- **Cause**: Communication data corruption
- **Risk**: Commands may not execute correctly
- **Response**: Servo disabled immediately
- **Action**: Check RS-485 wiring, termination, cable quality

#### Current Limit
- **Cause**: Motor drawing excessive current
- **Possible Reasons**:
  - Mechanical binding or obstruction
  - Motor fault or short circuit
  - Incorrect motor parameters
  - Overload condition
- **Response**: Servo disabled immediately
- **Action**: Investigate mechanical issue, check motor connections

#### Position Error
- **Cause**: Servo cannot maintain commanded position
- **Possible Reasons**:
  - Mechanical binding
  - Insufficient motor torque
  - Loss of encoder signal
  - Excessive load
  - Tuning problems
- **Response**: Servo disabled immediately
- **Action**: Check mechanical system, verify encoder operation, review servo tuning

#### Monitoring Faults

```hal
# Monitor global fault status
net ldcn-global-fault ldcn.fault => pyvcp.fault-led

# Monitor individual servo faults
net x-fault ldcn.servo.0.fault => pyvcp.x-fault-led
net y-fault ldcn.servo.1.fault => pyvcp.y-fault-led
net z-fault ldcn.servo.2.fault => pyvcp.z-fault-led
```

### 3. Watchdog Timer

#### Purpose

The watchdog timer provides protection against:
- LinuxCNC HAL thread failure
- Computer system crash or freeze
- Communication interface failure
- Software bugs

#### Configuration

Default timeout: 1.0 second

```hal
# Conservative setting for slower systems
setp ldcn.watchdog-timeout 2.0

# Aggressive setting for fast response (recommended)
setp ldcn.watchdog-timeout 0.5

# Disable watchdog (NOT RECOMMENDED)
setp ldcn.watchdog-timeout 0
```

#### Recommendations

- **Production Machines**: 0.5 - 1.0 seconds
- **Development/Testing**: 1.0 - 2.0 seconds
- **Slow Real-time Systems**: 2.0 seconds
- **Never disable** watchdog on production machines

#### What Happens on Watchdog Timeout

1. All servo amplifiers immediately disabled
2. `ldcn.comms-ok` set to FALSE
3. Error message logged to system log
4. Machine must be reset before re-enabling

### 4. Power-On Detection

#### I/O Controller Integration

The CNC-SK-2310g2 I/O controller monitors machine power state:
- Physical power button on control panel
- Monitors +24V power supply status
- Provides power-on status to HAL component

#### Power-On Requirements

Servos can only be enabled when:
```
ldcn.power-on == TRUE
```

This ensures:
- Machine operator has explicitly enabled power
- Power supply is stable and within specifications
- E-stop chain is reset and closed

#### Power-Off Response

When power is turned off (button or loss of supply):
1. `ldcn.power-on` immediately goes FALSE
2. All servo amplifiers disabled
3. LinuxCNC enters E-stop state
4. Tool head decelerates using servo brakes

### 5. Safe Enable Logic

Servos are enabled ONLY when ALL conditions are met:

```
ENABLE = power_on AND
         NOT fault AND
         NOT estop AND
         comms_ok AND
         enable_request
```

If ANY condition fails:
- Servos are immediately disabled
- System enters safe state
- Operator must investigate and correct issue

## Operational Safety Procedures

### Pre-Operation Checklist

Before EACH operating session:

- [ ] Inspect machine for damage or wear
- [ ] Verify E-stop buttons function correctly
- [ ] Check all safety guards are in place
- [ ] Verify work area is clear of people and obstructions
- [ ] Test communication with `ldcn_diagnostic.py`
- [ ] Verify power-on detection works
- [ ] Check servo enable/disable function
- [ ] Review G-code program for errors

### During Operation

**STAY ALERT** at all times:

- Keep hands away from moving parts
- Monitor machine operation continuously
- Keep E-stop button within immediate reach
- Watch for unusual sounds, vibrations, or behavior
- Be prepared to hit E-stop at any moment

**NEVER:**
- Reach into machine while it's running
- Bypass or disable safety features
- Leave machine unattended while running
- Operate machine while impaired or fatigued
- Allow untrained persons to operate machine

### After Operation

- [ ] Disable machine power
- [ ] Wait for all motion to stop completely
- [ ] Remove workpiece safely
- [ ] Clean machine and work area
- [ ] Report any abnormal behavior or issues

## Troubleshooting Safety Issues

### Frequent E-stop Activations

**Symptoms**: E-stop activates frequently during operation

**Possible Causes**:
- Mechanical binding or interference
- Servo tuning issues causing position errors
- Insufficient motor torque
- Communication errors
- Power supply problems

**Investigation**:
1. Run `ldcn_diagnostic.py` to check communication
2. Check `ldcn.servo.N.fault` pins to identify which servo
3. Examine mechanical system for binding
4. Review servo tuning parameters
5. Check power supply voltage and current capacity

**DO NOT**:
- Disable fault detection
- Increase position error limits excessively
- Continue operation with recurring faults

### Watchdog Timeout Errors

**Symptoms**: Servos disable with "Watchdog timeout" message

**Possible Causes**:
- Computer system too slow or overloaded
- Real-time scheduling problems
- Communication interface issues
- Excessive servo thread period

**Investigation**:
1. Check LinuxCNC latency test results
2. Verify real-time kernel is installed
3. Review system load (CPU, memory)
4. Check `ldcn.comm-errors` parameter
5. Inspect USB-RS485 adapter and cables

**Solutions**:
- Increase `ldcn.watchdog-timeout` temporarily for testing
- Optimize computer system (close unnecessary programs)
- Verify real-time configuration
- Replace suspect cables or adapters
- Use dedicated real-time computer

### Communication Errors

**Symptoms**: `ldcn.comms-ok` goes FALSE, `ldcn.comm-errors` increases

**Possible Causes**:
- RS-485 wiring issues
- Missing or incorrect termination
- Cable too long or poor quality
- Electrical noise/interference
- Damaged USB-RS485 adapter

**Investigation**:
1. Check RS-485 wiring (A, B, GND)
2. Verify 120Ω termination resistors at both ends
3. Measure cable length (<1000ft for 125kbps)
4. Check for nearby noise sources (VFDs, welders, etc.)
5. Inspect cable for damage

**Solutions**:
- Repair or replace wiring
- Add proper termination
- Use shielded twisted-pair cable
- Route away from noise sources
- Replace adapter if damaged

## Emergency Procedures

### Immediate Danger

**IF YOU SENSE IMMEDIATE DANGER:**

1. **HIT E-STOP IMMEDIATELY**
2. Move away from machine
3. Verify all motion has stopped
4. Cut main power if necessary
5. Do not reset until danger is eliminated

### Runaway Motion

**IF MACHINE MOVES UNEXPECTEDLY:**

1. **HIT E-STOP IMMEDIATELY**
2. Do not attempt to interfere with moving parts
3. Wait for complete stop
4. Investigate cause before resetting

**Possible Causes**:
- Software bug or G-code error
- Position error causing runaway
- Encoder failure
- Communication error sending wrong command

**DO NOT reset machine until cause is identified and corrected.**

### Fire or Smoke

**IF YOU SEE FIRE OR SMELL BURNING:**

1. **HIT E-STOP**
2. **CUT MAIN POWER**
3. Evacuate area
4. Call emergency services if fire spreads
5. Use appropriate fire extinguisher if safe to do so

**DO NOT:**
- Use water on electrical fires
- Open electrical enclosures while powered
- Attempt repairs until power is removed and verified

### Electrical Shock

**IF SOMEONE RECEIVES ELECTRICAL SHOCK:**

1. **DO NOT TOUCH THE PERSON**
2. Cut main power immediately
3. Call emergency services
4. If safe, move person away from electrical source using non-conductive object
5. Perform CPR if trained and necessary

**Prevention**:
- Ensure all electrical work meets codes
- Use proper grounding
- Install GFCI protection
- Keep electrical enclosures closed and locked
- Never work on live circuits

## Maintenance and Safety

### Regular Safety Inspections

**Weekly**:
- Test E-stop buttons
- Check safety guards
- Inspect wiring for damage
- Verify communication status

**Monthly**:
- Test all safety interlocks
- Check servo encoder operation
- Verify power supply voltages
- Review error logs

**Annually**:
- Professional safety audit
- Electrical system inspection
- Mechanical system inspection
- Update safety documentation

### Documentation

Maintain records of:
- Safety inspection results
- Fault conditions and resolutions
- Configuration changes
- Maintenance performed
- Operator training

## Legal and Liability

### Operator Responsibility

**YOU** are responsible for:
- Safe operation of the machine
- Following all safety procedures
- Maintaining safety systems
- Ensuring proper training
- Reporting safety issues

### Manufacturer Disclaimer

The LDCN HAL component is provided "AS IS" without warranty. The authors and contributors are not liable for:
- Injuries or deaths resulting from use
- Property damage from machine malfunction
- Consequential or incidental damages
- Fitness for any particular purpose

**YOU ASSUME ALL RISK** when using this software.

### Compliance

Ensure your machine complies with:
- Local electrical codes
- Workplace safety regulations
- OSHA requirements (USA)
- CE marking requirements (Europe)
- Relevant industry standards

Consult with qualified professionals for compliance verification.

## Support and Resources

### Getting Help

- **GitHub Issues**: https://github.com/ndemarco/linuxcnc-logosol/issues
- **LinuxCNC Forum**: https://forum.linuxcnc.org/
- **Emergency**: Contact professional service technician

### Additional Reading

- LinuxCNC Integrator Manual: http://linuxcnc.org/docs/html/integrator/integrator-concepts.html
- Machine Safety Standards: ISO 12100 (General Safety Principles)
- Emergency Stop Devices: ISO 13850
- Servo Drive Safety: IEC 61800-5-2

---

**REMEMBER**: Safety is YOUR responsibility. When in doubt, STOP and seek qualified assistance.

**Last Updated**: 2025-01-26
**Document Version**: 1.0
