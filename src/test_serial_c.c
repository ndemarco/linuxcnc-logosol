/*
 * Simple C test for LDCN serial communication (no HAL)
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ldcn_serial.h"
#include "ldcn_protocol.h"

int main(int argc, char **argv) {
    ldcn_serial_port_t *port;
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    const char *device = "/dev/ttyUSB0";

    printf("=== LDCN C Serial Test ===\n");
    printf("Opening %s at 19200 baud...\n", device);

    port = ldcn_serial_open(device, 19200);
    if (!port) {
        fprintf(stderr, "Failed to open serial port\n");
        return -1;
    }
    printf("✓ Port opened\n");

    // Wait a bit after opening (like Python code does)
    usleep(200000); // 200ms

    // Send HARD_RESET command
    printf("\nSending HARD_RESET...\n");
    ldcn_cmd_hard_reset(&cmd, LDCN_ADDR_BROADCAST);
    printf("  Packet: header=0x%02X addr=0x%02X cmd=0x%02X checksum=0x%02X\n",
           cmd.header, cmd.address, cmd.command, cmd.checksum);

    ldcn_serial_send_command(port, &cmd);
    printf("✓ Command sent\n");

    // Wait for devices to reset
    printf("Waiting 2 seconds for devices to reset...\n");
    sleep(2);

    // Try to read status from address 1
    printf("\nReading status from address 1...\n");
    ldcn_cmd_read_status(&cmd, 0x01, LDCN_STATUS_SEND_POS);
    int ret = ldcn_serial_exchange(port, &cmd, &status, 100);

    if (ret >= 0) {
        printf("✓ Got response! Status=0x%02X\n", status.status);
    } else {
        printf("✗ No response\n");
    }

    // Try SET_ADDRESS to address 0 (find first device)
    printf("\nSending SET_ADDRESS to broadcast...\n");
    ldcn_cmd_set_address(&cmd, 0x00, 1, 0xFF, false);
    ret = ldcn_serial_exchange(port, &cmd, &status, 200);

    if (ret >= 0) {
        printf("✓ Got response! Status=0x%02X\n", status.status);
    } else {
        printf("✗ No response\n");
    }

    ldcn_serial_close(port);
    printf("\n✓ Port closed\n");
    printf("=== Test complete ===\n");

    return 0;
}
