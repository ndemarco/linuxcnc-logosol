# Makefile for LinuxCNC Logosol LDCN Driver

# Detect LinuxCNC installation
LINUXCNC_DIR := /usr
HAL_INCLUDE := $(LINUXCNC_DIR)/include/linuxcnc

# Compiler and flags
CC := gcc
CFLAGS := -Wall -Wextra -O2 -g
CFLAGS += -I$(HAL_INCLUDE) -I.
CFLAGS += -DULAPI
LDFLAGS := -llinuxcnchal -lm

# Target binary
TARGET := ldcn

# Source files
SRCS := ldcn.c ldcn_protocol.c ldcn_serial.c
OBJS := $(SRCS:.c=.o)

# Installation directories
PREFIX ?= /usr/local
BINDIR := $(PREFIX)/bin
DOCDIR := $(PREFIX)/share/doc/linuxcnc-ldcn

# Default target
all: $(TARGET)

# Build the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Dependencies
ldcn.o: ldcn.c ldcn_protocol.h ldcn_serial.h
ldcn_protocol.o: ldcn_protocol.c ldcn_protocol.h
ldcn_serial.o: ldcn_serial.c ldcn_serial.h ldcn_protocol.h

# Install
install: $(TARGET)
	@echo "Installing $(TARGET) to $(BINDIR)"
	@install -d $(BINDIR)
	@install -m 755 $(TARGET) $(BINDIR)/
	@install -d $(DOCDIR)
	@install -m 644 ../README.md $(DOCDIR)/ 2>/dev/null || true
	@echo "Installation complete"
	@echo "Usage: loadusr -W ldcn port=/dev/ttyUSB0 baud=115200 axes=3"

# Uninstall
uninstall:
	@echo "Removing $(TARGET) from $(BINDIR)"
	@rm -f $(BINDIR)/$(TARGET)
	@rm -rf $(DOCDIR)
	@echo "Uninstall complete"

# Clean build artifacts
clean:
	@rm -f $(OBJS) $(TARGET)
	@echo "Clean complete"

# Clean everything including backup files
distclean: clean
	@rm -f *~ *.bak
	@echo "Distclean complete"

# Help
help:
	@echo "LinuxCNC Logosol LDCN Driver - Makefile targets:"
	@echo ""
	@echo "  make          - Build the driver"
	@echo "  make install  - Install the driver to $(BINDIR)"
	@echo "  make uninstall- Remove the installed driver"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make distclean- Remove all generated files"
	@echo "  make help     - Show this help message"
	@echo ""
	@echo "Configuration:"
	@echo "  PREFIX=$(PREFIX)"
	@echo "  BINDIR=$(BINDIR)"
	@echo "  LINUXCNC_DIR=$(LINUXCNC_DIR)"

.PHONY: all install uninstall clean distclean help
