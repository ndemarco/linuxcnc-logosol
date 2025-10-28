/*
 * ldcn_serial.h
 * 
 * Serial Communication Layer for LDCN
 * 
 * Copyright (C) 2024
 * License: GPL-2.0
 */

#ifndef LDCN_SERIAL_H
#define LDCN_SERIAL_H

#include "ldcn_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* Serial Port Handle */
typedef struct ldcn_serial_port ldcn_serial_port_t;

/* Open serial port */
ldcn_serial_port_t *ldcn_serial_open(const char *device, int baud_rate);

/* Close serial port */
void ldcn_serial_close(ldcn_serial_port_t *port);

/* Send command packet */
int ldcn_serial_send_command(ldcn_serial_port_t *port, const ldcn_cmd_packet_t *cmd);

/* Receive status packet */
int ldcn_serial_recv_status(ldcn_serial_port_t *port, ldcn_status_packet_t *status,
                           int timeout_ms);

/* Send command and wait for response */
int ldcn_serial_exchange(ldcn_serial_port_t *port,
                        const ldcn_cmd_packet_t *cmd,
                        ldcn_status_packet_t *status,
                        int timeout_ms);

/* Flush receive buffer */
void ldcn_serial_flush(ldcn_serial_port_t *port);

/* Set baud rate */
int ldcn_serial_set_baud(ldcn_serial_port_t *port, int baud_rate);

/* Get file descriptor (for select/poll) */
int ldcn_serial_get_fd(ldcn_serial_port_t *port);

/* Set debug level for serial communication
 * 0 = errors only
 * 1 = normal (high-level status messages)
 * 2 = verbose (show all TX/RX hex dumps)
 */
void ldcn_serial_set_debug(int level);

#endif /* LDCN_SERIAL_H */
