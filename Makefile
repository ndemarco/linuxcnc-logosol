# Makefile for LDCN HAL Component
#
# This Makefile builds the LDCN HAL component for LinuxCNC
#
# Usage:
#   make              - Build the HAL component
#   make install      - Install to LinuxCNC
#   make clean        - Clean build files
#   make test         - Run tests
#
# Requirements:
#   - LinuxCNC development packages (linuxcnc-dev)
#   - gcc compiler
#   - make

# Component name
COMP_NAME = ldcn

# Source files
SOURCES = src/ldcn_protocol.c src/ldcn_hal.c
HEADERS = src/ldcn_protocol.h

# LinuxCNC directories
# Try to auto-detect LinuxCNC installation
LINUXCNC_DIR ?= /usr
LINUXCNC_INC = $(LINUXCNC_DIR)/include/linuxcnc
LINUXCNC_LIB = $(LINUXCNC_DIR)/lib

# Compiler flags
CC = gcc
CFLAGS = -Wall -Wextra -O2 -fPIC
CFLAGS += -I$(LINUXCNC_INC)
CFLAGS += -DRTAPI -D_GNU_SOURCE

# Linker flags
LDFLAGS = -shared
LDFLAGS += -L$(LINUXCNC_LIB)
LDFLAGS += -Wl,-rpath,$(LINUXCNC_LIB)

# Output
TARGET = $(COMP_NAME).so

# Build rules
.PHONY: all clean install uninstall test help

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	@echo "Building LDCN HAL component..."
	$(CC) $(CFLAGS) $(SOURCES) $(LDFLAGS) -o $(TARGET)
	@echo "Build complete: $(TARGET)"

clean:
	@echo "Cleaning build files..."
	rm -f $(TARGET)
	rm -f *.o
	@echo "Clean complete"

install: $(TARGET)
	@echo "Installing LDCN HAL component..."
	@if [ ! -d "$(LINUXCNC_LIB)/linuxcnc/modules" ]; then \
		echo "Error: LinuxCNC lib directory not found at $(LINUXCNC_LIB)"; \
		echo "Please set LINUXCNC_DIR variable or check LinuxCNC installation"; \
		exit 1; \
	fi
	install -m 755 $(TARGET) $(LINUXCNC_LIB)/linuxcnc/modules/
	@echo "Install complete"
	@echo ""
	@echo "To use the component, add to your HAL file:"
	@echo "  loadrt ldcn port=/dev/ttyUSB0 num_servos=5"
	@echo "  addf ldcn.read servo-thread"

uninstall:
	@echo "Uninstalling LDCN HAL component..."
	rm -f $(LINUXCNC_LIB)/linuxcnc/modules/$(TARGET)
	@echo "Uninstall complete"

test:
	@echo "Running tests..."
	@if [ -f tests/test_protocol ]; then \
		tests/test_protocol; \
	else \
		echo "Tests not built yet"; \
	fi

# Make utilities executable
utils:
	chmod +x utils/*.py
	@echo "Utilities are now executable"

help:
	@echo "LDCN HAL Component Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make              - Build the HAL component"
	@echo "  make install      - Install to LinuxCNC"
	@echo "  make uninstall    - Remove from LinuxCNC"
	@echo "  make clean        - Clean build files"
	@echo "  make test         - Run tests"
	@echo "  make utils        - Make utility scripts executable"
	@echo "  make help         - Show this help"
	@echo ""
	@echo "Variables:"
	@echo "  LINUXCNC_DIR      - LinuxCNC installation directory (default: /usr)"
	@echo ""
	@echo "Example:"
	@echo "  make LINUXCNC_DIR=/usr/local"
	@echo "  sudo make install"

# Show build information
info:
	@echo "Build Information:"
	@echo "  Component:     $(COMP_NAME)"
	@echo "  Target:        $(TARGET)"
	@echo "  LinuxCNC Dir:  $(LINUXCNC_DIR)"
	@echo "  Include Dir:   $(LINUXCNC_INC)"
	@echo "  Library Dir:   $(LINUXCNC_LIB)"
	@echo "  Compiler:      $(CC)"
	@echo "  C Flags:       $(CFLAGS)"
	@echo "  LD Flags:      $(LDFLAGS)"
