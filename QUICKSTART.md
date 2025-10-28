# Quick Start Guide - LinuxCNC Logosol LDCN Driver

Get up and running in under 10 minutes.

## Step 1: Build (2 minutes)

```bash
git clone https://github.com/ndemarco/linuxcnc-logosol.git
cd linuxcnc-logosol/src
make
sudo make install
```

## Step 2: Test Connection (1 minute)

```bash
# Check USB adapter
dmesg | grep FTDI
ls -l /dev/ttyUSB0

# Add permissions if needed
sudo usermod -a -G dialout $USER
# Then log out and back in
```

## Step 3: Test Driver (2 minutes)

**Option A: Quick Test Script**
```bash
cd src
./test_init.sh
# Watch for initialization messages, press Ctrl+C to stop
```

**Option B: Manual Test with halrun**
```bash
cd src
halrun -I
# In halrun prompt:
halcmd: loadusr -W ldcn port=/dev/ttyUSB0 baud=125000 axes=1
halcmd: show pin ldcn
# Press Ctrl+D to exit
```

**What You Should See:**
```
ldcn: Opening /dev/ttyUSB0 at 19200 baud (LDCN default)
ldcn: Resetting all LDCN devices...
ldcn: Initializing 1 drives at 19200 baud...
ldcn: Initializing axis 0 (addr 0x01)
ldcn: Axis 0 initialized successfully
ldcn: Upgrading communication speed to 125000 baud...
ldcn: Verifying communication at 125000 baud...
ldcn: Successfully upgraded to 125000 baud
ldcn: Entering main loop
```

## Step 4: Calculate Your Scale (2 minutes)

```
SCALE = encoder_counts_per_rev / pitch_per_rev

Example: 2000 count encoder, 5mm ballscrew
SCALE = 2000 / 5 = 400 counts/mm
```

## Step 5: Configure (2 minutes)

Edit your HAL file:

```hal
# Driver initializes at 19200, then upgrades to 125000 for better performance
loadusr -W ldcn port=/dev/ttyUSB0 baud=125000 axes=3

setp ldcn.0.scale 400.0  # Your calculated value
setp ldcn.0.kp 50
setp ldcn.0.kd 8000

net x-enable joint.0.amp-enable-out => ldcn.0.enable
net x-pos-cmd joint.0.motor-pos-cmd => ldcn.0.position-cmd
net x-pos-fb ldcn.0.position-fb => joint.0.motor-pos-fb
```

## Step 6: Run LinuxCNC (1 minute)

```bash
linuxcnc your_config.ini
```

## Common Issues

**Wrong direction?**
```hal
setp ldcn.0.scale -400.0  # Negate the scale
```

**Following error?**
```hal
setp ldcn.0.kp 100  # Increase gain
```

**Motor oscillates?**
```hal
setp ldcn.0.kp 25   # Reduce gain
setp ldcn.0.kd 12000  # Increase damping
```

## Next Steps

- See README.md for full documentation
- See TECHNICAL.md for tuning guide
- See examples/ for complete configurations

Happy machining!
