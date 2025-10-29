#!/bin/bash
# Monitor E-stop status in real-time

echo "Starting LDCN driver..."
echo "This test will skip power-wait and attempt to monitor E-stop"
echo ""

# Start halrun in background
halrun -U > /dev/null 2>&1
sleep 1

# Load component with explicit init phases (skip power wait)
halcmd loadusr -W ./ldcn port=/dev/ttyUSB0 baud=125000 axes=1 2>&1 | grep -E "(ERROR|Initialized|estop|din)" &
LDCN_PID=$!

sleep 10

echo "Driver should be loaded. Checking pins..."
echo ""

# Show all pins
halcmd show pin ldcn 2>/dev/null
echo ""

# Monitor E-stop pins in a loop
echo "Monitoring E-stop status (Ctrl+C to exit):"
echo "============================================"
while true; do
    ESTOP_OK=$(halcmd getp ldcn.estop-ok 2>/dev/null)
    DIN_RAW=$(halcmd getp ldcn.din-raw 2>/dev/null)

    echo -ne "\rE-stop OK: $ESTOP_OK  |  DIN Raw: $DIN_RAW  "
    sleep 0.5
done
