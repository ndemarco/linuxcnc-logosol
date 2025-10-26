# LDCN Troubleshooting Guide

This guide helps diagnose and resolve common issues with the LDCN HAL component.

## Quick Diagnostic Checklist

Run these checks in order:

1. **Check USB adapter**: `ls -l /dev/ttyUSB*`
2. **Test permissions**: `sudo chmod 666 /dev/ttyUSB0`
3. **Run diagnostics**: `./utils/ldcn_diagnostic.py`
4. **Check wiring**: Verify A, B, GND connections
5. **Monitor status**: `./utils/ldcn_monitor.py`

## Common Issues

### No Devices Found

**Symptom**: `ldcn_diagnostic.py` reports "No devices found at any common baud rate"

**Causes and Solutions**:

1. **USB adapter not connected**
   ```bash
   ls -l /dev/ttyUSB*
   ```
   - If no `/dev/ttyUSB0`, check USB connection
   - Try different USB port
   - Check `dmesg` for USB errors

2. **Wrong serial port**
   ```bash
   ls -l /dev/tty* | grep USB
   ```
   - May be `/dev/ttyUSB1` or `/dev/ttyACM0`
   - Specify correct port: `--port /dev/ttyUSB1`

3. **Permission denied**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # Or add user to dialout group
   sudo usermod -a -G dialout $USER
   # Then log out and back in
   ```

4. **RS-485 wiring issues**
   - Check A and B are not swapped
   - Verify GND connections
   - Test cable continuity
   - Check for short circuits

5. **Devices powered off**
   - Verify 24V power to servo drives
   - Check power LEDs on devices
   - Measure voltage at terminals

6. **Wrong baud rate**
   ```bash
   # Try explicit baud rates
   ./utils/ldcn_diagnostic.py --verbose
   ```
   - Watch for any response at any baud rate
   - Devices may be at non-standard baud rate

### Device 6 (I/O Controller) Not Responding

**Symptom**: Servos 1-5 respond, but device 6 never responds

**Causes and Solutions**:

1. **I/O controller needs more time**
   - 2310g2 can be slower to respond
   - Increase delays in addressing sequence
   - Already handled in `ldcn_init.py`

2. **I/O controller at different address**
   ```bash
   # Try addresses 1-10
   for i in {1..10}; do
     echo "Testing address $i"
     python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0',19200,timeout=0.1); s.write(b'\\xaa' + bytes([i]) + b'\\x0e' + bytes([(i+0x0e)&0xff])); print(s.read(10).hex())"
   done
   ```

3. **I/O controller configuration issue**
   - May need to configure status reporting first
   - Try full status: `ldcn_init.py` handles this

### Power Not Detected

**Symptom**: `ldcn.power-on` pin always FALSE, even after pressing power button

**Diagnostic Steps**:

1. **Verify I/O controller responding**
   ```bash
   ./utils/ldcn_diagnostic.py
   ```
   - If device 6 not responding, fix that first

2. **Test power detection manually**
   ```bash
   ./utils/ldcn_diagnostic.py --test-power
   ```
   - Follow prompts
   - Should see status change from 0x04 to 0x0C

3. **Check J21 jumper**
   - On 2310g2, J21 controls power button mode
   - Should be OPEN for physical button
   - If SHORT, controller expects software control

4. **Verify status byte format**
   ```bash
   ./utils/ldcn_monitor.py --interval 0.5
   ```
   - Watch status byte before/after power on
   - Should change: 0x04 → 0x0C (bit 3 toggles)

5. **Check full status configuration**
   ```python
   # Manually configure I/O controller
   import serial
   s = serial.Serial('/dev/ttyUSB0', 125000, timeout=0.2)
   # Define status: request all data (0xFFFF)
   s.write(bytes([0xAA, 0x06, 0x22, 0xFF, 0xFF, 0x26]))
   ```

### Communication Errors

**Symptom**: Frequent errors, `ldcn.comms-ok` goes FALSE intermittently

**Causes and Solutions**:

1. **Cable quality issues**
   - Check for damaged cable
   - Verify shielding intact
   - Replace cable if old

2. **Termination resistors**
   - Need 120Ω at both ends
   - Check with multimeter
   - Measure ~60Ω across A-B with all devices powered on

3. **Cable too long**
   - Maximum recommended: 1000 ft at 125kbps
   - Reduce baud rate for longer cables:
     ```bash
     ./utils/ldcn_init.py --target-baud 57600
     ```

4. **Electrical noise**
   - Move RS-485 cable away from AC power
   - Use shielded cable
   - Connect cable shield to GND at one end only

5. **Multiple masters**
   - Only one device should be master (PC)
   - Check no other software accessing serial port
   - Check for competing HAL components

### Servos Won't Enable

**Symptom**: `ldcn.servo.N.enabled` pins stay FALSE

**Diagnostic Steps**:

1. **Check power state**
   ```bash
   halcmd show pin ldcn.power-on
   ```
   - Must be TRUE before servos can enable
   - Press physical power button if FALSE

2. **Check enable request**
   ```bash
   halcmd show pin ldcn.enable-request
   ```
   - Should be TRUE when LinuxCNC is enabled
   - Check HAL connection to motion.motion-enabled

3. **Manually test enable**
   ```bash
   halcmd setp ldcn.enable-request TRUE
   halcmd show pin ldcn.servo
   ```

4. **Check servo drive faults**
   - Look for fault LEDs on drives
   - Check motor connections
   - Verify encoder connections

5. **Communication working?**
   ```bash
   halcmd show pin ldcn.comms-ok
   ```
   - Must be TRUE for commands to work

### Baud Rate Change Fails

**Symptom**: Network responds at 19200 but not after switching to 125kbps

**Diagnostic Steps**:

1. **Verify BRD value**
   ```python
   # Check BRD calculation
   baud = 125000
   BRD = 0x27  # Should be 0x27 for 125kbps
   print(f"BRD for {baud}: 0x{BRD:02X}")
   ```

2. **Check timing**
   - Must wait 500ms after baud command
   - Must physically close/reopen serial port
   - Must wait 500ms after reopening

3. **USB adapter limitations**
   - Some cheap adapters don't support high baud rates
   - Test with known-good adapter
   - Try lower baud rate (57600)

4. **Try manual baud change**
   ```bash
   # At 19200
   ./utils/ldcn_init.py --target-baud 19200
   
   # Then test at 125k
   ./utils/ldcn_diagnostic.py --verbose
   ```

### HAL Component Won't Load

**Symptom**: `loadrt ldcn` fails in LinuxCNC

**Diagnostic Steps**:

1. **Check installation**
   ```bash
   ls -l /usr/lib/linuxcnc/modules/ldcn.so
   ```
   - If missing: `sudo make install`

2. **Check dependencies**
   ```bash
   ldd /usr/lib/linuxcnc/modules/ldcn.so
   ```
   - All libraries should be found

3. **Check permissions**
   ```bash
   ls -l /dev/ttyUSB0
   ```
   - Should be readable/writable
   - LinuxCNC runs as user, not root

4. **Check dmesg for errors**
   ```bash
   sudo dmesg | grep ldcn
   ```

5. **Test component manually**
   ```bash
   halrun
   loadrt ldcn port=/dev/ttyUSB0 num_servos=5
   show
   ```

## Advanced Debugging

### Enable Verbose Logging

```bash
# Add to HAL file
loadusr -W halcmd show pin ldcn > /tmp/ldcn_debug.log
```

### Monitor RS-485 Traffic

```bash
# Install interceptty
sudo apt-get install interceptty

# Intercept serial port
interceptty /dev/ttyUSB0 /tmp/interceptty &

# Watch traffic
cat /tmp/interceptty
```

### Check Real-Time Performance

```bash
halcmd show thread
halcmd show funct ldcn.read
```

### Capture Communication

```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 125000, timeout=0.2)

# Send NOP and capture response
cmd = bytes([0xAA, 0x01, 0x0E, 0x0F])
print(f"TX: {cmd.hex()}")
ser.write(cmd)
time.sleep(0.02)
resp = ser.read(50)
print(f"RX: {resp.hex()}")
```

## Hardware Debugging

### Test RS-485 Adapter

```bash
# Loopback test (connect A to A, B to B)
python3 << EOF
import serial
s = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
s.write(b'TEST')
print(s.read(4))  # Should read back 'TEST'
EOF
```

### Check Voltage Levels

- Measure A-B differential: Should be ±2V to ±6V idle
- Check 24V supply to drives
- Verify logic levels at drive terminals

### Verify Termination

```bash
# With all devices powered ON, measure resistance across A-B
# Should read approximately 60 ohms (two 120Ω in parallel)
```

## Getting Help

If issues persist:

1. **Gather information**:
   ```bash
   ./utils/ldcn_diagnostic.py --verbose > ldcn_debug.txt 2>&1
   dmesg > dmesg.txt
   halcmd show all > hal_config.txt
   ```

2. **Create issue on GitHub**: Include:
   - Hardware configuration
   - Output of diagnostic script
   - dmesg output
   - HAL configuration

3. **LinuxCNC Forum**: Post with:
   - Complete error messages
   - What you've tried
   - Hardware details

## Preventive Maintenance

- **Clean RS-485 terminals** periodically
- **Check cable for wear** near moving parts
- **Verify termination resistors** annually
- **Test backup** USB adapter
- **Document working configuration** for future reference

## Known Issues

### Issue: Slow initialization at startup

**Workaround**: Run `ldcn_init.py` before starting LinuxCNC

### Issue: Occasional communication timeout

**Workaround**: Increase servo thread period slightly

### Issue: USB adapter resets on power cycle

**Workaround**: Create udev rule for consistent device naming

## Reference

For more details, see:
- [PROTOCOL.md](PROTOCOL.md) - Complete protocol documentation
- [README.md](../README.md) - Usage guide
- LinuxCNC HAL manual - General HAL debugging
