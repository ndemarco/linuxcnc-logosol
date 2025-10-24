#!/bin/bash
# Test script for LDCN connection

echo "========================================="
echo "LDCN Connection Test Script"
echo "========================================="
echo ""

# Check for FTDI device
echo "1. Checking for USB adapter..."
if dmesg | grep -i FTDI | tail -1 | grep -q "attached"; then
    echo "   ✓ FTDI USB adapter detected"
    DEVICE=$(dmesg | grep "FTDI.*attached to" | tail -1 | sed 's/.*attached to //')
    echo "     Device: /dev/$DEVICE"
else
    echo "   ❌ No FTDI adapter found"
    exit 1
fi

# Check device exists
echo ""
echo "2. Checking device file..."
if [ -e "/dev/$DEVICE" ]; then
    echo "   ✓ Device file exists: /dev/$DEVICE"
else
    echo "   ❌ Device file not found"
    exit 1
fi

# Check permissions
echo ""
echo "3. Checking permissions..."
if [ -r "/dev/$DEVICE" ] && [ -w "/dev/$DEVICE" ]; then
    echo "   ✓ Read/write access OK"
else
    echo "   ❌ No access to /dev/$DEVICE"
    echo "   Fix with: sudo usermod -a -G dialout $USER"
    exit 1
fi

# Check if ldcn driver exists
echo ""
echo "4. Checking for ldcn driver..."
if [ -f "../src/ldcn" ]; then
    echo "   ✓ Driver found"
elif command -v ldcn &> /dev/null; then
    echo "   ✓ Driver found in PATH"
else
    echo "   ❌ Driver not found"
    echo "      Build with: cd ../src && make"
    exit 1
fi

echo ""
echo "========================================="
echo "✓ ALL TESTS PASSED"
echo "========================================="
echo ""
echo "Your LDCN hardware is ready to use!"
