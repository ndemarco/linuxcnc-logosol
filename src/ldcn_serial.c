/*
 * ldcn_serial.c
 * 
 * Serial Communication Layer Implementation
 * 
 * Copyright (C) 2024
 * License: GPL-2.0
 */

#include "ldcn_serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

/* Serial Port Structure */
struct ldcn_serial_port {
    int fd;
    char device[256];
    int baud_rate;
    struct termios orig_termios;
};

/* Convert baud rate integer to termios constant */
static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        /* LDCN-specific baud rates - use B38400 as placeholder for custom baud */
        case 125000:
        case 312500:
        case 625000:
        case 1250000:
            return B38400;  /* Will set actual baud with TIOCSSERIAL */
        default:
            fprintf(stderr, "Warning: unsupported baud rate %d, using 115200\n", baud);
            return B115200;
    }
}

/* Open and configure serial port */
ldcn_serial_port_t *ldcn_serial_open(const char *device, int baud_rate) {
    ldcn_serial_port_t *port;
    struct termios tio;
    
    /* Allocate port structure */
    port = calloc(1, sizeof(ldcn_serial_port_t));
    if (!port) {
        perror("malloc");
        return NULL;
    }
    
    /* Store device and baud rate */
    strncpy(port->device, device, sizeof(port->device) - 1);
    port->baud_rate = baud_rate;
    
    /* Open serial device (use O_NONBLOCK initially to avoid blocking on DCD) */
    port->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (port->fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", device, strerror(errno));
        free(port);
        return NULL;
    }

    /* Clear O_NONBLOCK for proper blocking I/O */
    int flags = fcntl(port->fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(port->fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    /* Get current terminal settings */
    if (tcgetattr(port->fd, &port->orig_termios) < 0) {
        perror("tcgetattr");
        close(port->fd);
        free(port);
        return NULL;
    }
    
    /* Configure terminal for raw 8N1 mode - match Python pyserial exactly */
    memset(&tio, 0, sizeof(tio));

    /* Use exact same flags as Python pyserial */
    tio.c_iflag = 0;        /* iflag = 0x00000000 */
    tio.c_oflag = 0;        /* oflag = 0x00000000 */
    tio.c_cflag = 0x08be;   /* cflag = 0x000008be (from Python) */
    tio.c_lflag = 0;        /* lflag = 0x00000000 */

    /* Control characters */
    tio.c_cc[VMIN] = 0;   /* Non-blocking read */
    tio.c_cc[VTIME] = 0;  /* No timeout */
    
    /* Set baud rate */
    speed_t speed = baud_to_speed(baud_rate);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    
    /* Apply settings */
    if (tcsetattr(port->fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr");
        close(port->fd);
        free(port);
        return NULL;
    }

    /* Set custom baud rate for LDCN-specific rates using FTDI ioctl */
    if (baud_rate == 125000 || baud_rate == 312500 ||
        baud_rate == 625000 || baud_rate == 1250000) {
        struct serial_struct serial;

        if (ioctl(port->fd, TIOCGSERIAL, &serial) < 0) {
            fprintf(stderr, "Warning: failed to get serial info for custom baud: %s\n",
                    strerror(errno));
            fprintf(stderr, "         continuing with standard baud rate\n");
        } else {
            serial.flags = (serial.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
            serial.custom_divisor = (serial.baud_base + (baud_rate / 2)) / baud_rate;

            if (ioctl(port->fd, TIOCSSERIAL, &serial) < 0) {
                fprintf(stderr, "Warning: failed to set custom baud %d: %s\n",
                        baud_rate, strerror(errno));
                fprintf(stderr, "         continuing with standard baud rate\n");
            }
        }
    }

    /* Flush any existing data */
    tcflush(port->fd, TCIOFLUSH);

    /* Wait for port to stabilize (like Python pyserial does) */
    usleep(200000);  /* 200ms */

    return port;
}

/* Close serial port */
void ldcn_serial_close(ldcn_serial_port_t *port) {
    if (!port) return;
    
    /* Restore original terminal settings */
    if (port->fd >= 0) {
        tcsetattr(port->fd, TCSANOW, &port->orig_termios);
        close(port->fd);
    }
    
    free(port);
}

/* Send command packet */
int ldcn_serial_send_command(ldcn_serial_port_t *port, const ldcn_cmd_packet_t *cmd) {
    if (!port || !cmd) return -1;

    /* Build packet to send: header + address + command + data + checksum */
    uint8_t buffer[LDCN_MAX_DATA_BYTES + 4];
    int len = 0;

    buffer[len++] = cmd->header;
    buffer[len++] = cmd->address;
    buffer[len++] = cmd->command;

    for (int i = 0; i < cmd->data_len; i++) {
        buffer[len++] = cmd->data[i];
    }

    buffer[len++] = cmd->checksum;

    /* Debug: print hex dump */
    fprintf(stderr, "TX (%d bytes): ", len);
    for (int i = 0; i < len; i++) {
        fprintf(stderr, "%02x", buffer[i]);
    }
    fprintf(stderr, "\n");

    /* Send packet */
    int sent = write(port->fd, buffer, len);
    if (sent < 0) {
        perror("write");
        return -1;
    }

    if (sent != len) {
        fprintf(stderr, "Warning: partial write (%d of %d bytes)\n", sent, len);
    }

    fprintf(stderr, "  -> wrote %d bytes to fd=%d\n", sent, port->fd);

    /* Drain output to ensure transmission */
    tcdrain(port->fd);

    return sent;
}

/* Receive status packet with timeout */
int ldcn_serial_recv_status(ldcn_serial_port_t *port, ldcn_status_packet_t *status,
                           int timeout_ms) {
    if (!port || !status) return -1;
    
    fd_set readfds;
    struct timeval tv;
    int ret;
    uint8_t byte;
    int bytes_read = 0;
    
    /* Clear status structure */
    memset(status, 0, sizeof(ldcn_status_packet_t));
    
    /* Set timeout */
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    /* Read status byte */
    FD_ZERO(&readfds);
    FD_SET(port->fd, &readfds);
    
    ret = select(port->fd + 1, &readfds, NULL, NULL, &tv);
    if (ret <= 0) {
        return (ret == 0) ? -2 : -1;  /* -2 = timeout */
    }
    
    if (read(port->fd, &status->status, 1) != 1) {
        return -1;
    }
    bytes_read = 1;
    
    /* 
     * Read optional data bytes
     * Note: We need to know how many bytes to expect based on
     * the status configuration. For now, read until timeout or
     * we get a reasonable checksum.
     * 
     * Better approach: track expected status packet length
     */
    status->data_len = 0;
    
    /* Read up to LDCN_MAX_STATUS_BYTES or until timeout */
    for (int i = 0; i < LDCN_MAX_STATUS_BYTES + 1; i++) {
        tv.tv_sec = 0;
        tv.tv_usec = 5000;  /* 5ms between bytes */
        
        FD_ZERO(&readfds);
        FD_SET(port->fd, &readfds);
        
        ret = select(port->fd + 1, &readfds, NULL, NULL, &tv);
        if (ret <= 0) {
            /* Timeout - assume previous byte was checksum */
            if (status->data_len > 0) {
                status->checksum = status->data[status->data_len - 1];
                status->data_len--;
            } else {
                /* Only status byte received, next byte should be checksum */
                if (read(port->fd, &status->checksum, 1) == 1) {
                    bytes_read++;
                }
            }
            break;
        }
        
        if (read(port->fd, &byte, 1) == 1) {
            status->data[status->data_len++] = byte;
            bytes_read++;
        }
    }

    /* Debug: print hex dump of received data */
    if (bytes_read > 0) {
        fprintf(stderr, "RX (%d bytes): ", bytes_read);
        fprintf(stderr, "%02x", status->status);
        for (int i = 0; i < status->data_len; i++) {
            fprintf(stderr, "%02x", status->data[i]);
        }
        if (bytes_read > status->data_len + 1) {
            fprintf(stderr, "%02x", status->checksum);
        }
        fprintf(stderr, "\n");
    }

    return bytes_read;
}

/* Send command and receive response */
int ldcn_serial_exchange(ldcn_serial_port_t *port,
                        const ldcn_cmd_packet_t *cmd,
                        ldcn_status_packet_t *status,
                        int timeout_ms) {
    int ret;
    
    /* Flush receive buffer */
    ldcn_serial_flush(port);
    
    /* Send command */
    ret = ldcn_serial_send_command(port, cmd);
    if (ret < 0) {
        return ret;
    }
    
    /* Wait for command to be transmitted */
    tcdrain(port->fd);
    
    /* Receive response */
    ret = ldcn_serial_recv_status(port, status, timeout_ms);
    
    return ret;
}

/* Flush receive buffer */
void ldcn_serial_flush(ldcn_serial_port_t *port) {
    if (!port || port->fd < 0) return;
    tcflush(port->fd, TCIFLUSH);
}

/* Change baud rate */
int ldcn_serial_set_baud(ldcn_serial_port_t *port, int baud_rate) {
    struct termios tio;
    
    if (!port || port->fd < 0) return -1;
    
    /* Get current settings */
    if (tcgetattr(port->fd, &tio) < 0) {
        perror("tcgetattr");
        return -1;
    }
    
    /* Set new baud rate */
    speed_t speed = baud_to_speed(baud_rate);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    
    /* Apply settings */
    if (tcsetattr(port->fd, TCSANOW, &tio) < 0) {
        perror("tcsetattr");
        return -1;
    }

    /* Set custom baud rate for LDCN-specific rates using FTDI ioctl */
    if (baud_rate == 125000 || baud_rate == 312500 ||
        baud_rate == 625000 || baud_rate == 1250000) {
        struct serial_struct serial;

        if (ioctl(port->fd, TIOCGSERIAL, &serial) < 0) {
            fprintf(stderr, "Warning: failed to get serial info for custom baud: %s\n",
                    strerror(errno));
            fprintf(stderr, "         continuing with standard baud rate\n");
        } else {
            serial.flags = (serial.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
            serial.custom_divisor = (serial.baud_base + (baud_rate / 2)) / baud_rate;

            if (ioctl(port->fd, TIOCSSERIAL, &serial) < 0) {
                fprintf(stderr, "Warning: failed to set custom baud %d: %s\n",
                        baud_rate, strerror(errno));
                fprintf(stderr, "         continuing with standard baud rate\n");
            }
        }
    }

    port->baud_rate = baud_rate;

    /* Flush buffers */
    tcflush(port->fd, TCIOFLUSH);

    return 0;
}

/* Get file descriptor */
int ldcn_serial_get_fd(ldcn_serial_port_t *port) {
    return port ? port->fd : -1;
}
