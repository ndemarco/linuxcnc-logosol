# Documentation Index

This directory contains all project documentation organized by category.

---

## Directory Structure

### `/docs/logosol/` - Logosol Hardware Documentation
Official PDF manuals and datasheets for Logosol LDCN devices.

**Contents:**
- `CNC_Multi-Axis_Servo_Controller.pdf` - Multi-axis controller overview
- `LS-231SE-Advanced-Multifunctional-Servo-Drive.pdf` - LS-231SE servo drive manual
- `LS-2310g2-Supervisor-IO-Controller.pdf` - SK-2310g2 supervisor manual
- `LS-2315-High-Performance-Spindle-Drive.pdf` - Spindle drive manual
- `LS-773-Network-IO-Node.pdf` - Network I/O node manual
- `LS-832RL-USB-to-LDCN-Node-Isolated-Converter.pdf` - USB converter manual

### `/docs/ldcn_module/` - Python Module Development Documentation
Planning and design documents for the LDCN Python module refactoring project.

**Contents:**
- `LDCN_MODULE_PLAN.md` - Overall project plan with phases and milestones
- `PYTHON_FILE_INVENTORY.md` - Inventory of existing Python utilities
- `FUNCTION_CATEGORIZATION.md` - Function mapping and verification status

**Purpose:** Track the development of a reusable, object-oriented Python module for LDCN network communication.

### Root Documentation Files
Project-specific documentation for LinuxCNC integration.

**Contents:**
- `PROTOCOL.md` - LDCN protocol implementation notes
- `SAFETY.md` - Safety considerations for CNC operation
- `TROUBLESHOOTING.md` - Common issues and solutions
- `estop-and-power-control.md` - E-stop and power button implementation
- `spindle-and-io.md` - Spindle control and I/O configuration
- `LINUX_SETUP.md` - Linux environment setup notes

---

## Quick Links

### For Hardware Reference
→ See `/docs/logosol/` for official Logosol manuals

### For Python Module Development
→ See `/docs/ldcn_module/LDCN_MODULE_PLAN.md` for project roadmap
→ See `/docs/ldcn_module/FUNCTION_CATEGORIZATION.md` for implementation status

### For LinuxCNC Integration
→ See `PROTOCOL.md` for LDCN communication details
→ See `SAFETY.md` for safety system design
→ See `TROUBLESHOOTING.md` for problem solving

---

## Document Status

**Last Updated:** 2025-10-29

**Recent Changes:**
- Reorganized documentation structure (2025-10-29)
- Moved Logosol PDFs to `/logosol/` subdirectory
- Created `/ldcn_module/` for Python module development docs
- Added verification tracking to function categorization
