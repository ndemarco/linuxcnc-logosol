#!/bin/bash
# Quick test script for LDCN driver initialization

echo "========================================"
echo "LDCN Driver Initialization Test"
echo "========================================"
echo ""
echo "Testing initialization sequence:"
echo "  1. Open at 19200 baud"
echo "  2. Reset devices"
echo "  3. Initialize drives"
echo "  4. Upgrade to 125000 baud"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================"
echo ""

# Run with halrun to set up HAL environment
halrun -I test_ldcn.hal
