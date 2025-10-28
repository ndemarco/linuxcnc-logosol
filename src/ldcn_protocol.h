/*
 * ldcn_protocol.h - Logosol Distributed Control Network (LDCN) Protocol
 * 
 * This file defines the LDCN serial communication protocol for Logosol
 * servo drives and I/O controllers.
 * 
 * Copyright (C) 2025
 * License: GPL v2 or later
 */

#ifndef LDCN_PROTOCOL_H
#define LDCN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol Constants */
#define LDCN_HEADER 0xAA
#define LDCN_MAX_DEVICES 6
#define LDCN_MAX_DATA_BYTES 16
#define LDCN_MAX_RESPONSE 50

/* Device Types */
#define LDCN_DEVICE_SERVO 0    /* LS-231SE Servo Drive */
#define LDCN_DEVICE_IO 1       /* CNC-SK-2310g2 I/O Controller */

/* Command Codes */
#define LDCN_CMD_RESET_POSITION 0x0
#define LDCN_CMD_SET_ADDRESS    0x1
#define LDCN_CMD_DEFINE_STATUS  0x2
#define LDCN_CMD_READ_STATUS    0x3
#define LDCN_CMD_LOAD_TRAJECTORY 0x4
#define LDCN_CMD_START_MOTION   0x5
#define LDCN_CMD_SET_GAIN       0x6
#define LDCN_CMD_STOP_MOTOR     0x7
#define LDCN_CMD_IO_CONTROL     0x8
#define LDCN_CMD_SET_HOME_MODE  0x9
#define LDCN_CMD_SET_BAUD_RATE  0xA
#define LDCN_CMD_CLEAR_BITS     0xB
#define LDCN_CMD_SAVE_HOME      0xC
#define LDCN_CMD_ADD_PATH_POINTS 0xD
#define LDCN_CMD_NOP            0xE
#define LDCN_CMD_HARD_RESET     0xF

/* Special Addresses */
#define LDCN_ADDR_UNADDRESSED 0x00
#define LDCN_ADDR_GROUP_ALL   0xFF

/* Baud Rate Divisors (BRD) */
#define LDCN_BRD_9600    0x81
#define LDCN_BRD_19200   0x3F
#define LDCN_BRD_57600   0x14
#define LDCN_BRD_115200  0x0A
#define LDCN_BRD_125000  0x27
#define LDCN_BRD_312500  0x0F
#define LDCN_BRD_625000  0x07
#define LDCN_BRD_1250000 0x03

/* Status Byte Bits (Servo Drives) */
#define LDCN_STATUS_MOVE_DONE     (1 << 0)
#define LDCN_STATUS_CKSUM_ERROR   (1 << 1)
#define LDCN_STATUS_CURRENT_LIMIT (1 << 2)
#define LDCN_STATUS_POWER_ON      (1 << 3)
#define LDCN_STATUS_POS_ERROR     (1 << 4)
#define LDCN_STATUS_HOME_SOURCE   (1 << 5)
#define LDCN_STATUS_LIMIT2        (1 << 6)
#define LDCN_STATUS_HOME_IN_PROG  (1 << 7)

/* Auxiliary Status Byte Bits */
#define LDCN_AUX_INDEX         (1 << 0)
#define LDCN_AUX_POS_WRAP      (1 << 1)
#define LDCN_AUX_SERVO_ON      (1 << 2)
#define LDCN_AUX_ACCEL_DONE    (1 << 3)
#define LDCN_AUX_SLEW_DONE     (1 << 4)
#define LDCN_AUX_SERVO_OVERRUN (1 << 5)
#define LDCN_AUX_PATH_MODE     (1 << 6)

/* Stop Motor Command Bits */
#define LDCN_STOP_ABRUPTLY      (1 << 0)
#define LDCN_STOP_SMOOTHLY      (1 << 1)
#define LDCN_STOP_MOTOR_OFF     (1 << 2)
#define LDCN_STOP_AMP_ENABLE    (1 << 4)

/* Define Status Bits */
#define LDCN_STATUS_SEND_POSITION    (1 << 0)
#define LDCN_STATUS_SEND_AD          (1 << 1)
#define LDCN_STATUS_SEND_VELOCITY    (1 << 2)
#define LDCN_STATUS_SEND_AUX         (1 << 3)
#define LDCN_STATUS_SEND_HOME        (1 << 4)
#define LDCN_STATUS_SEND_DEVICE_ID   (1 << 5)
#define LDCN_STATUS_SEND_POS_ERROR   (1 << 6)
#define LDCN_STATUS_SEND_PATH_BUFFER (1 << 7)
#define LDCN_STATUS_SEND_DIGITAL_IN  (1 << 8)
#define LDCN_STATUS_SEND_ANALOG_IN   (1 << 9)
#define LDCN_STATUS_SEND_WATCHDOG    (1 << 12)
#define LDCN_STATUS_SEND_MOTOR_POS   (1 << 13)

/* Timeouts (milliseconds) */
#define LDCN_TIMEOUT_COMMAND 100
#define LDCN_TIMEOUT_RESET   2000
#define LDCN_TIMEOUT_ADDRESS 300

/* Error Codes */
#define LDCN_OK                  0
#define LDCN_ERR_OPEN_FAILED    -1
#define LDCN_ERR_NO_RESPONSE    -2
#define LDCN_ERR_CHECKSUM       -3
#define LDCN_ERR_TIMEOUT        -4
#define LDCN_ERR_INVALID_PARAM  -5
#define LDCN_ERR_COMM_FAILED    -6

/**
 * LDCN Device Structure
 */
typedef struct {
    uint8_t address;        /* Individual address (1-127) */
    uint8_t group_address;  /* Group address (128-255) */
    uint8_t device_type;    /* LDCN_DEVICE_SERVO or LDCN_DEVICE_IO */
    uint8_t status;         /* Last status byte */
    uint8_t aux_status;     /* Last auxiliary status byte */
    bool responding;        /* Device responded to last command */
} ldcn_device_t;

/**
 * LDCN Network Structure
 */
typedef struct {
    int fd;                                /* Serial port file descriptor */
    int baud_rate;                         /* Current baud rate */
    char port_name[256];                   /* Serial port device name */
    ldcn_device_t devices[LDCN_MAX_DEVICES]; /* Device array */
    int num_devices;                       /* Number of addressed devices */
    int comm_errors;                       /* Communication error count */
    uint8_t last_command[LDCN_MAX_RESPONSE]; /* Last command sent */
    int last_command_len;                  /* Length of last command */
} ldcn_network_t;

/* Maximum status data bytes */
#define LDCN_MAX_STATUS_BYTES 32

/* Status Packet Structure (received from device) */
typedef struct {
    uint8_t status;                          /* Status byte */
    uint8_t data[LDCN_MAX_STATUS_BYTES];    /* Optional status data */
    uint8_t data_len;                        /* Actual data length */
    uint8_t checksum;                        /* Sum of status+data */
} __attribute__((packed)) ldcn_status_packet_t;

/* Drive Status Structure (parsed from status packet) */
typedef struct {
    /* Status byte flags */
    bool move_done;
    bool checksum_error;
    bool current_limit;
    bool power_on;
    bool position_error;
    bool home_in_progress;

    /* Auxiliary status byte flags */
    bool servo_on;
    bool path_mode;
    bool servo_overrun;

    /* Optional data fields */
    int32_t position;           /* Encoder position */
    int16_t velocity;           /* Actual velocity */
    int32_t home_position;      /* Home position */
    int16_t following_error;    /* Position error */
    uint8_t buffer_count;       /* Path points in buffer */
    uint8_t ad_value;           /* A/D value */
    uint16_t digital_inputs;    /* Digital input status (2 bytes) */

    /* Device info */
    uint8_t device_id;
    uint8_t version;
} ldcn_drive_status_t;

/* PID Gain Parameters */
typedef struct {
    uint16_t kp;            /* Position gain (0-0x7FFF) */
    uint16_t kd;            /* Velocity gain (0-0x7FFF) */
    uint16_t ki;            /* Integral gain (0-0x7FFF) */
    uint16_t il;            /* Integration limit (0-0x7FFF) */
    uint8_t  ol;            /* Output limit (0-0xFF) */
    uint8_t  cl;            /* Current limit (0-0xFF, odd values) */
    uint16_t el;            /* Position error limit (0-0x3FFF) */
    uint8_t  sr;            /* Servo rate divisor (1-0xFF) */
    uint8_t  db;            /* Deadband (reserved) */
} ldcn_gain_params_t;

/* Trajectory Parameters */
typedef struct {
    int32_t position;       /* Goal position (encoder counts) */
    uint32_t velocity;      /* Velocity (counts/tick * 65536) */
    uint32_t acceleration;  /* Accel (counts/tick² * 65536) */
    uint16_t pwm;          /* PWM value (0-255) */
    bool servo_mode;        /* true=position, false=PWM */
    bool velocity_mode;     /* true=velocity, false=trapezoid */
    bool direction_fwd;     /* true=forward, false=reverse */
    bool start_now;         /* Execute immediately */
} ldcn_trajectory_t;

/* Function Prototypes */

/**
 * ldcn_open - Open serial port and initialize network structure
 * @port: Serial port device name (e.g., "/dev/ttyUSB0")
 * @baud_rate: Initial baud rate (typically 19200)
 * 
 * Returns: Pointer to ldcn_network_t on success, NULL on failure
 */
ldcn_network_t *ldcn_open(const char *port, int baud_rate);

/**
 * ldcn_close - Close serial port and free network structure
 * @net: Network structure
 */
void ldcn_close(ldcn_network_t *net);

/**
 * ldcn_auto_detect_baud - Detect current baud rate of network
 * @port: Serial port device name
 * 
 * Returns: Detected baud rate, or -1 if no devices found
 */
int ldcn_auto_detect_baud(const char *port);

/**
 * ldcn_hard_reset - Send hard reset to all devices
 * @net: Network structure
 * 
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_hard_reset(ldcn_network_t *net);

/**
 * ldcn_address_devices - Address all devices on the network
 * @net: Network structure
 * @num_devices: Number of devices to address (1-6)
 * 
 * Returns: Number of devices successfully addressed, or error code
 */
int ldcn_address_devices(ldcn_network_t *net, int num_devices);

/**
 * ldcn_set_baud_rate - Change baud rate of all devices
 * @net: Network structure
 * @new_baud: New baud rate (must be a standard LDCN baud rate)
 * 
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_set_baud_rate(ldcn_network_t *net, int new_baud);

/**
 * ldcn_send_command - Send a command to a device
 * @net: Network structure
 * @address: Device address
 * @command: Command code
 * @data: Data bytes (can be NULL)
 * @data_len: Length of data
 * @response: Buffer for response (can be NULL)
 * @response_len: Size of response buffer
 * 
 * Returns: Number of response bytes received, or error code
 */
int ldcn_send_command(ldcn_network_t *net, uint8_t address, uint8_t command,
                      const uint8_t *data, int data_len,
                      uint8_t *response, int response_len);

/**
 * ldcn_read_status - Read status from a device
 * @net: Network structure
 * @address: Device address
 * @status: Pointer to store status byte
 * 
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_read_status(ldcn_network_t *net, uint8_t address, uint8_t *status);

/**
 * ldcn_stop_motor - Stop a servo motor
 * @net: Network structure
 * @address: Servo drive address
 * @mode: Stop mode (LDCN_STOP_* flags)
 * 
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_stop_motor(ldcn_network_t *net, uint8_t address, uint8_t mode);

/**
 * ldcn_enable_amplifier - Enable or disable servo amplifier
 * @net: Network structure
 * @address: Servo drive address
 * @enable: true to enable, false to disable
 * 
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_enable_amplifier(ldcn_network_t *net, uint8_t address, bool enable);

/**
 * ldcn_define_status - Configure what status data to return
 * @net: Network structure
 * @address: Device address
 * @status_bits: Status bits to request (LDCN_STATUS_SEND_* flags)
 * 
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_define_status(ldcn_network_t *net, uint8_t address, uint16_t status_bits);

/**
 * ldcn_verify_devices - Verify all devices are responding
 * @net: Network structure
 * 
 * Returns: Number of responding devices
 */
int ldcn_verify_devices(ldcn_network_t *net);

/**
 * ldcn_get_brd_for_baud - Get BRD value for a baud rate
 * @baud_rate: Desired baud rate
 * 
 * Returns: BRD value, or -1 if unsupported
 */
int ldcn_get_brd_for_baud(int baud_rate);

/**
 * ldcn_calculate_checksum - Calculate LDCN checksum
 * @data: Data bytes (excluding header)
 * @len: Length of data
 *
 * Returns: Checksum byte
 */
uint8_t ldcn_calculate_checksum(const uint8_t *data, int len);

/**
 * ldcn_emergency_stop - Immediately stop all servo motors
 * @net: Network structure
 *
 * Sends emergency stop command to all servo drives simultaneously.
 * This disables all amplifiers and stops all motion abruptly.
 *
 * Returns: LDCN_OK on success, error code on failure
 */
int ldcn_emergency_stop(ldcn_network_t *net);

/**
 * ldcn_check_faults - Check for fault conditions on a device
 * @net: Network structure
 * @address: Device address
 * @faults: Pointer to store fault flags (bitwise OR of detected faults)
 *
 * Reads device status and checks for fault conditions:
 * - LDCN_STATUS_CKSUM_ERROR: Checksum error
 * - LDCN_STATUS_CURRENT_LIMIT: Current limit reached
 * - LDCN_STATUS_POS_ERROR: Position error exceeded
 *
 * Returns: Number of faults detected, or error code (<0) on communication failure
 */
int ldcn_check_faults(ldcn_network_t *net, uint8_t address, uint8_t *faults);

/**
 * ldcn_parse_status - Parse status packet into structured data
 * @pkt: Status packet received from device
 * @status: Pointer to drive status structure to populate
 * @expected_fields: Bitwise OR of LDCN_STATUS_SEND_* flags indicating which fields are present
 *
 * Parses a status packet from a servo drive into a structured format.
 * The expected_fields parameter must match the fields configured via ldcn_define_status.
 *
 * Returns: true on success, false on invalid parameters
 */
bool ldcn_parse_status(const ldcn_status_packet_t *pkt, ldcn_drive_status_t *status,
                       uint16_t expected_fields);

#endif /* LDCN_PROTOCOL_H */
