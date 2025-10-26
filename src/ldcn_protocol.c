/*
 * ldcn_protocol.c - Logosol Distributed Control Network (LDCN) Protocol Implementation
 * 
 * This file implements the LDCN serial communication protocol for Logosol
 * servo drives and I/O controllers.
 * 
 * Copyright (C) 2025
 * License: GPL v2 or later
 */

#include "ldcn_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

/* Internal helper function prototypes */
static int ldcn_open_serial(const char *port, int baud_rate);
static int ldcn_set_serial_baud(int fd, int baud_rate);
static int ldcn_read_with_timeout(int fd, uint8_t *buf, int len, int timeout_ms);
static int ldcn_try_communicate(int fd, uint8_t address);
static void ldcn_flush_input(int fd);

/**
 * ldcn_calculate_checksum - Calculate LDCN checksum
 */
uint8_t ldcn_calculate_checksum(const uint8_t *data, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

/**
 * ldcn_get_brd_for_baud - Get BRD value for a baud rate
 */
int ldcn_get_brd_for_baud(int baud_rate)
{
    switch (baud_rate) {
        case 9600:    return LDCN_BRD_9600;
        case 19200:   return LDCN_BRD_19200;
        case 57600:   return LDCN_BRD_57600;
        case 115200:  return LDCN_BRD_115200;
        case 125000:  return LDCN_BRD_125000;
        case 312500:  return LDCN_BRD_312500;
        case 625000:  return LDCN_BRD_625000;
        case 1250000: return LDCN_BRD_1250000;
        default:      return -1;
    }
}

/**
 * ldcn_open_serial - Open and configure serial port
 */
static int ldcn_open_serial(const char *port, int baud_rate)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("ldcn_open_serial: open");
        return LDCN_ERR_OPEN_FAILED;
    }

    if (ldcn_set_serial_baud(fd, baud_rate) < 0) {
        close(fd);
        return LDCN_ERR_OPEN_FAILED;
    }

    return fd;
}

/**
 * ldcn_set_serial_baud - Configure serial port baud rate
 */
static int ldcn_set_serial_baud(int fd, int baud_rate)
{
    struct termios tty;
    
    if (tcgetattr(fd, &tty) < 0) {
        perror("ldcn_set_serial_baud: tcgetattr");
        return -1;
    }

    /* Convert baud rate to termios constant */
    speed_t speed;
    switch (baud_rate) {
        case 9600:    speed = B9600; break;
        case 19200:   speed = B19200; break;
        case 38400:   speed = B38400; break;
        case 57600:   speed = B57600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        default:
            fprintf(stderr, "ldcn_set_serial_baud: Unsupported baud rate %d\n", baud_rate);
            return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* 8N1 mode, no flow control */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     /* 8-bit chars */
    tty.c_iflag &= ~IGNBRK;                         /* disable break processing */
    tty.c_lflag = 0;                                /* no signaling chars, no echo, no canonical processing */
    tty.c_oflag = 0;                                /* no remapping, no delays */
    tty.c_cc[VMIN]  = 0;                            /* read doesn't block */
    tty.c_cc[VTIME] = 5;                            /* 0.5 seconds read timeout */

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         /* shut off xon/xoff ctrl */
    tty.c_cflag |= (CLOCAL | CREAD);                /* ignore modem controls, enable reading */
    tty.c_cflag &= ~(PARENB | PARODD);              /* shut off parity */
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("ldcn_set_serial_baud: tcsetattr");
        return -1;
    }

    return 0;
}

/**
 * ldcn_flush_input - Flush input buffer
 */
static void ldcn_flush_input(int fd)
{
    tcflush(fd, TCIFLUSH);
}

/**
 * ldcn_read_with_timeout - Read from serial port with timeout
 */
static int ldcn_read_with_timeout(int fd, uint8_t *buf, int len, int timeout_ms)
{
    fd_set readfds;
    struct timeval tv;
    int n = 0;
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    
    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0) {
        n = read(fd, buf, len);
    }
    
    return n;
}

/**
 * ldcn_try_communicate - Try to communicate with a device
 */
static int ldcn_try_communicate(int fd, uint8_t address)
{
    uint8_t cmd[4];
    uint8_t response[10];
    
    /* Send No Op command */
    cmd[0] = LDCN_HEADER;
    cmd[1] = address;
    cmd[2] = (0 << 4) | LDCN_CMD_NOP;  /* 0 data bytes, NOP command */
    cmd[3] = ldcn_calculate_checksum(&cmd[1], 2);
    
    ldcn_flush_input(fd);
    
    if (write(fd, cmd, 4) != 4) {
        return LDCN_ERR_COMM_FAILED;
    }
    
    tcdrain(fd);  /* Wait for output to complete */
    usleep(10000); /* 10ms delay */
    
    int n = ldcn_read_with_timeout(fd, response, sizeof(response), 100);
    
    return (n >= 2) ? LDCN_OK : LDCN_ERR_NO_RESPONSE;
}

/**
 * ldcn_auto_detect_baud - Detect current baud rate of network
 */
int ldcn_auto_detect_baud(const char *port)
{
    int baud_rates[] = {19200, 125000, 115200, 57600, 9600, 38400, 0};
    uint8_t test_addresses[] = {1, 2, 3, 6};
    int num_test_addr = sizeof(test_addresses) / sizeof(test_addresses[0]);
    
    for (int i = 0; baud_rates[i] != 0; i++) {
        int fd = ldcn_open_serial(port, baud_rates[i]);
        if (fd < 0) {
            continue;
        }
        
        usleep(200000); /* 200ms settle time */
        
        /* Try to communicate with several addresses */
        for (int j = 0; j < num_test_addr; j++) {
            if (ldcn_try_communicate(fd, test_addresses[j]) == LDCN_OK) {
                close(fd);
                return baud_rates[i];
            }
        }
        
        close(fd);
    }
    
    return -1;
}

/**
 * ldcn_open - Open serial port and initialize network structure
 */
ldcn_network_t *ldcn_open(const char *port, int baud_rate)
{
    ldcn_network_t *net = calloc(1, sizeof(ldcn_network_t));
    if (!net) {
        return NULL;
    }

    net->fd = ldcn_open_serial(port, baud_rate);
    if (net->fd < 0) {
        free(net);
        return NULL;
    }

    net->baud_rate = baud_rate;
    net->num_devices = 0;
    net->comm_errors = 0;

    /* Store port name for later use */
    strncpy(net->port_name, port, sizeof(net->port_name) - 1);
    net->port_name[sizeof(net->port_name) - 1] = '\0';

    /* Initialize devices */
    for (int i = 0; i < LDCN_MAX_DEVICES; i++) {
        net->devices[i].address = 0;
        net->devices[i].group_address = LDCN_ADDR_GROUP_ALL;
        net->devices[i].device_type = LDCN_DEVICE_SERVO;
        net->devices[i].responding = false;
    }

    usleep(200000); /* 200ms settle time */

    return net;
}

/**
 * ldcn_close - Close serial port and free network structure
 */
void ldcn_close(ldcn_network_t *net)
{
    if (net) {
        if (net->fd >= 0) {
            close(net->fd);
        }
        free(net);
    }
}

/**
 * ldcn_send_command - Send a command to a device
 */
int ldcn_send_command(ldcn_network_t *net, uint8_t address, uint8_t command,
                      const uint8_t *data, int data_len,
                      uint8_t *response, int response_len)
{
    if (!net || net->fd < 0) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    if (data_len > LDCN_MAX_DATA_BYTES) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    /* Build command packet */
    uint8_t packet[LDCN_MAX_RESPONSE];
    packet[0] = LDCN_HEADER;
    packet[1] = address;
    packet[2] = ((data_len & 0x0F) << 4) | (command & 0x0F);
    
    if (data_len > 0) {
        memcpy(&packet[3], data, data_len);
    }
    
    packet[3 + data_len] = ldcn_calculate_checksum(&packet[1], 2 + data_len);
    
    int packet_len = 4 + data_len;
    
    /* Store last command for debugging */
    memcpy(net->last_command, packet, packet_len);
    net->last_command_len = packet_len;
    
    /* Flush input buffer */
    ldcn_flush_input(net->fd);
    
    /* Send command */
    int n = write(net->fd, packet, packet_len);
    if (n != packet_len) {
        net->comm_errors++;
        return LDCN_ERR_COMM_FAILED;
    }
    
    tcdrain(net->fd);
    usleep(10000); /* 10ms delay for device to process */
    
    /* Read response if buffer provided */
    if (response && response_len > 0) {
        n = ldcn_read_with_timeout(net->fd, response, response_len, LDCN_TIMEOUT_COMMAND);
        if (n < 0) {
            net->comm_errors++;
            return LDCN_ERR_TIMEOUT;
        }
        return n;
    }
    
    return LDCN_OK;
}

/**
 * ldcn_hard_reset - Send hard reset to all devices
 */
int ldcn_hard_reset(ldcn_network_t *net)
{
    if (!net || net->fd < 0) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    uint8_t packet[4];
    packet[0] = LDCN_HEADER;
    packet[1] = LDCN_ADDR_GROUP_ALL;
    packet[2] = (0 << 4) | LDCN_CMD_HARD_RESET;
    packet[3] = ldcn_calculate_checksum(&packet[1], 2);
    
    ldcn_flush_input(net->fd);
    
    int n = write(net->fd, packet, 4);
    if (n != 4) {
        return LDCN_ERR_COMM_FAILED;
    }
    
    tcdrain(net->fd);
    usleep(LDCN_TIMEOUT_RESET * 1000); /* Wait for reset to complete */
    
    ldcn_flush_input(net->fd);
    
    return LDCN_OK;
}

/**
 * ldcn_address_devices - Address all devices on the network
 */
int ldcn_address_devices(ldcn_network_t *net, int num_devices)
{
    if (!net || net->fd < 0) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    if (num_devices < 1 || num_devices > LDCN_MAX_DEVICES) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    int addressed = 0;
    
    for (int i = 1; i <= num_devices; i++) {
        uint8_t data[2];
        uint8_t response[10];
        
        data[0] = i;                    /* Individual address */
        data[1] = LDCN_ADDR_GROUP_ALL;  /* Group address */
        
        int n = ldcn_send_command(net, LDCN_ADDR_UNADDRESSED, LDCN_CMD_SET_ADDRESS,
                                  data, 2, response, sizeof(response));
        
        if (n >= 2) {
            net->devices[addressed].address = i;
            net->devices[addressed].group_address = LDCN_ADDR_GROUP_ALL;
            net->devices[addressed].responding = true;
            
            /* Device 6 is I/O controller, others are servo drives */
            net->devices[addressed].device_type = (i == 6) ? LDCN_DEVICE_IO : LDCN_DEVICE_SERVO;
            
            addressed++;
        }
        
        usleep(LDCN_TIMEOUT_ADDRESS * 1000);
    }
    
    net->num_devices = addressed;
    
    /* Additional stabilization time */
    usleep(2000000); /* 2 seconds */
    
    return addressed;
}

/**
 * ldcn_set_baud_rate - Change baud rate of all devices
 */
int ldcn_set_baud_rate(ldcn_network_t *net, int new_baud)
{
    if (!net || net->fd < 0) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    int brd = ldcn_get_brd_for_baud(new_baud);
    if (brd < 0) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    uint8_t data[1];
    data[0] = brd;
    
    /* Send to group address (all devices) */
    int ret = ldcn_send_command(net, LDCN_ADDR_GROUP_ALL, LDCN_CMD_SET_BAUD_RATE,
                                data, 1, NULL, 0);
    
    if (ret < 0) {
        return ret;
    }
    
    usleep(500000); /* 500ms for baud rate change */

    /* Close and reopen at new baud rate */
    close(net->fd);
    usleep(500000);

    net->fd = ldcn_open_serial(net->port_name, new_baud);
    if (net->fd < 0) {
        return LDCN_ERR_OPEN_FAILED;
    }

    net->baud_rate = new_baud;
    usleep(500000);

    return LDCN_OK;
}

/**
 * ldcn_read_status - Read status from a device
 */
int ldcn_read_status(ldcn_network_t *net, uint8_t address, uint8_t *status)
{
    if (!net || !status) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    uint8_t response[LDCN_MAX_RESPONSE];
    
    int n = ldcn_send_command(net, address, LDCN_CMD_NOP, NULL, 0,
                              response, sizeof(response));
    
    if (n < 2) {
        return LDCN_ERR_NO_RESPONSE;
    }
    
    /* For I/O controller with full status, status byte is at index 1 */
    /* For servo drives with minimal status, it's at index 0 */
    if (n > 2 && address == 6) {
        *status = response[1];
    } else {
        *status = response[0];
    }
    
    /* Update device status */
    for (int i = 0; i < net->num_devices; i++) {
        if (net->devices[i].address == address) {
            net->devices[i].status = *status;
            net->devices[i].responding = true;
            break;
        }
    }
    
    return LDCN_OK;
}

/**
 * ldcn_stop_motor - Stop a servo motor
 */
int ldcn_stop_motor(ldcn_network_t *net, uint8_t address, uint8_t mode)
{
    if (!net) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    uint8_t data[1];
    data[0] = mode;
    
    return ldcn_send_command(net, address, LDCN_CMD_STOP_MOTOR,
                            data, 1, NULL, 0);
}

/**
 * ldcn_enable_amplifier - Enable or disable servo amplifier
 */
int ldcn_enable_amplifier(ldcn_network_t *net, uint8_t address, bool enable)
{
    uint8_t mode;
    
    if (enable) {
        /* Stop abruptly with amplifier enabled */
        mode = LDCN_STOP_ABRUPTLY | LDCN_STOP_AMP_ENABLE;
    } else {
        /* Stop abruptly with amplifier disabled (motor off) */
        mode = LDCN_STOP_ABRUPTLY | LDCN_STOP_MOTOR_OFF;
    }
    
    return ldcn_stop_motor(net, address, mode);
}

/**
 * ldcn_define_status - Configure what status data to return
 */
int ldcn_define_status(ldcn_network_t *net, uint8_t address, uint16_t status_bits)
{
    if (!net) {
        return LDCN_ERR_INVALID_PARAM;
    }
    
    uint8_t data[2];
    data[0] = status_bits & 0xFF;
    data[1] = (status_bits >> 8) & 0xFF;
    
    return ldcn_send_command(net, address, LDCN_CMD_DEFINE_STATUS,
                            data, 2, NULL, 0);
}

/**
 * ldcn_verify_devices - Verify all devices are responding
 */
int ldcn_verify_devices(ldcn_network_t *net)
{
    if (!net) {
        return LDCN_ERR_INVALID_PARAM;
    }

    int responding = 0;

    for (int i = 0; i < net->num_devices; i++) {
        uint8_t status;
        int ret = ldcn_read_status(net, net->devices[i].address, &status);

        if (ret == LDCN_OK) {
            net->devices[i].responding = true;
            responding++;
        } else {
            net->devices[i].responding = false;
        }
    }

    return responding;
}

/**
 * ldcn_emergency_stop - Immediately stop all servo motors
 */
int ldcn_emergency_stop(ldcn_network_t *net)
{
    if (!net) {
        return LDCN_ERR_INVALID_PARAM;
    }

    /* Send emergency stop to all servos using group address */
    /* Stop abruptly with amplifiers disabled and motors off */
    uint8_t mode = LDCN_STOP_ABRUPTLY | LDCN_STOP_MOTOR_OFF;

    int ret = ldcn_stop_motor(net, LDCN_ADDR_GROUP_ALL, mode);

    /* Also disable each individual servo to ensure they're all off */
    for (int i = 0; i < net->num_devices; i++) {
        if (net->devices[i].device_type == LDCN_DEVICE_SERVO) {
            ldcn_enable_amplifier(net, net->devices[i].address, false);
        }
    }

    return ret;
}

/**
 * ldcn_check_faults - Check for fault conditions on a device
 */
int ldcn_check_faults(ldcn_network_t *net, uint8_t address, uint8_t *faults)
{
    if (!net || !faults) {
        return LDCN_ERR_INVALID_PARAM;
    }

    uint8_t status;
    int ret = ldcn_read_status(net, address, &status);

    if (ret != LDCN_OK) {
        return ret;
    }

    *faults = 0;
    int fault_count = 0;

    /* Check for checksum error */
    if (status & LDCN_STATUS_CKSUM_ERROR) {
        *faults |= LDCN_STATUS_CKSUM_ERROR;
        fault_count++;
    }

    /* Check for current limit */
    if (status & LDCN_STATUS_CURRENT_LIMIT) {
        *faults |= LDCN_STATUS_CURRENT_LIMIT;
        fault_count++;
    }

    /* Check for position error */
    if (status & LDCN_STATUS_POS_ERROR) {
        *faults |= LDCN_STATUS_POS_ERROR;
        fault_count++;
    }

    /* Update device status */
    for (int i = 0; i < net->num_devices; i++) {
        if (net->devices[i].address == address) {
            net->devices[i].status = status;
            break;
        }
    }

    return fault_count;
}
