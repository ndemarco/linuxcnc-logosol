# Linux-Specific Setup for LDCN Driver

## Critical: FTDI Latency Timer

**This is the most common issue when using LDCN devices on Linux!**

### Problem

The Linux FTDI driver (`ftdi_sio`) defaults to a 16ms latency timer. This delay is **way too long** for the LDCN protocol, which requires fast turnaround times for command/response cycles. With the default 16ms latency, the LDCN devices will appear unresponsive.

### Symptoms

- Devices work fine on Windows but not on Linux
- No responses from LDCN devices
- No relay clicks when sending HARD_RESET
- Python/C code can write to serial port but receives no data
- `dmesg` shows: `ftdi_sio ttyUSB0: use of SPD flags is deprecated`

### Solution

Set the FTDI latency timer to **1ms** (minimum):

```bash
# Check current latency (will show 16 by default)
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Set to 1ms (fixes the problem!)
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Verify it's set
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### Make It Permanent

Create a udev rule to automatically set the latency timer on device connection:

```bash
sudo nano /etc/udev/rules.d/99-ftdi-ldcn.rules
```

Add this line:

```
# Set FTDI latency timer to 1ms for LDCN communication
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

Reload udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now when you plug in the LS-832RL USB adapter, the latency will automatically be set to 1ms.

### Why This Happens

- **Windows**: FTDI drivers typically use 1-2ms latency by default
- **Linux**: ftdi_sio kernel module uses 16ms latency by default (for compatibility with slower devices)
- **LDCN Protocol**: Requires quick command/response cycles (typically <5ms)

The 16ms latency means data sits in the USB buffer for up to 16ms before being sent to/from the device, which breaks real-time communication protocols like LDCN.

### Verification

After setting the latency timer to 1ms, you should immediately hear relay clicks when running initialization commands:

```bash
python3 utilities/ldcn_initialization_test.py
# Should hear clicks during HARD_RESET phase
```

### Additional USB Optimization (Optional)

For even better performance, you can disable USB autosuspend for the FTDI device:

```bash
# Find the USB device ID
lsusb | grep FTDI
# Bus 003 Device 004: ID 0403:6001 Future Technology Devices International, Ltd FT232 Serial (UART) IC

# Disable autosuspend (replace Bus and Device numbers)
sudo sh -c 'echo -1 > /sys/bus/usb/devices/3-4/power/autosuspend_delay_ms'
```

## Summary

**Always set FTDI latency timer to 1ms when using LDCN devices on Linux!**

This is a one-time setup that will save you hours of debugging.
