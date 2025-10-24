/*
 * ldcn_protocol.h
 * 
 * Logosol LDCN Protocol Definitions
 * 
 * This file defines the LDCN communication protocol for
 * Logosol servo drives (LS-231SE, LS-2310g2)
 * 
 * Copyright (C) 2024
 * License: GPL-2.0
 */

#ifndef LDCN_PROTOCOL_H
#define LDCN_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol Constants */
#define LDCN_HEADER             0xAA
#define LDCN_MAX_DATA_BYTES     15
#define LDCN_MAX_STATUS_BYTES   23
#define LDCN_MAX_AXES           31
#define LDCN_PATH_BUFFER_SIZE   256

/* Default Values */
#define LDCN_DEFAULT_BAUD       19200
#define LDCN_DEFAULT_SERVO_RATE 1      /* 51.2 µsec */
#define LDCN_DEFAULT_GROUP_ADDR 0xFF

/* Address Ranges */
#define LDCN_ADDR_INDIVIDUAL_MIN 0x01
#define LDCN_ADDR_INDIVIDUAL_MAX 0x7F
#define LDCN_ADDR_GROUP_MIN      0x80
#define LDCN_ADDR_GROUP_MAX      0xFF
#define LDCN_ADDR_BROADCAST      0xFF
#define LDCN_ADDR_DEFAULT        0x00

/* Command Codes */
typedef enum {
    LDCN_CMD_RESET_POS      = 0x0,  /* Reset position counter */
    LDCN_CMD_SET_ADDR       = 0x1,  /* Set individual/group address */
    LDCN_CMD_DEFINE_STATUS  = 0x2,  /* Define status packet contents */
    LDCN_CMD_READ_STATUS    = 0x3,  /* Read status (one-time) */
    LDCN_CMD_LOAD_TRAJ      = 0x4,  /* Load trajectory parameters */
    LDCN_CMD_START_MOTION   = 0x5,  /* Execute loaded trajectory */
    LDCN_CMD_SET_GAIN       = 0x6,  /* Set PID gains and limits */
    LDCN_CMD_STOP_MOTOR     = 0x7,  /* Stop motor (various modes) */
    LDCN_CMD_IO_CONTROL     = 0x8,  /* I/O and path buffer control */
    LDCN_CMD_SET_HOME_MODE  = 0x9,  /* Set homing conditions */
    LDCN_CMD_SET_BAUD       = 0xA,  /* Set baud rate */
    LDCN_CMD_CLEAR_BITS     = 0xB,  /* Clear sticky status bits */
    LDCN_CMD_SAVE_HOME      = 0xC,  /* Save current pos as home */
    LDCN_CMD_ADD_PATH_PTS   = 0xD,  /* Add path points to buffer */
    LDCN_CMD_NOP            = 0xE,  /* No operation */
    LDCN_CMD_HARD_RESET     = 0xF   /* Hard reset */
} ldcn_cmd_t;

/* Status Byte Bits */
#define LDCN_STATUS_MOVE_DONE       (1 << 0)
#define LDCN_STATUS_CKSUM_ERROR     (1 << 1)
#define LDCN_STATUS_CURRENT_LIMIT   (1 << 2)
#define LDCN_STATUS_POWER_ON        (1 << 3)
#define LDCN_STATUS_POS_ERROR       (1 << 4)
#define LDCN_STATUS_HOME_SOURCE     (1 << 5)
#define LDCN_STATUS_LIMIT2          (1 << 6)
#define LDCN_STATUS_HOME_IN_PROG    (1 << 7)

/* Auxiliary Status Byte Bits */
#define LDCN_AUX_STATUS_INDEX       (1 << 0)
#define LDCN_AUX_STATUS_POS_WRAP    (1 << 1)
#define LDCN_AUX_STATUS_SERVO_ON    (1 << 2)
#define LDCN_AUX_STATUS_ACCEL_DONE  (1 << 3)
#define LDCN_AUX_STATUS_SLEW_DONE   (1 << 4)
#define LDCN_AUX_STATUS_SERVO_OVERRUN (1 << 5)
#define LDCN_AUX_STATUS_PATH_MODE   (1 << 6)

/* Define Status Bits (what to include in status packet) */
#define LDCN_STATUS_SEND_POS        (1 << 0)   /* 4 bytes */
#define LDCN_STATUS_SEND_AD         (1 << 1)   /* 1 byte */
#define LDCN_STATUS_SEND_VEL        (1 << 2)   /* 2 bytes */
#define LDCN_STATUS_SEND_AUX        (1 << 3)   /* 1 byte */
#define LDCN_STATUS_SEND_HOME       (1 << 4)   /* 4 bytes */
#define LDCN_STATUS_SEND_ID_VER     (1 << 5)   /* 2 bytes */
#define LDCN_STATUS_SEND_POS_ERR    (1 << 6)   /* 2 bytes */
#define LDCN_STATUS_SEND_BUFFER_CNT (1 << 7)   /* 1 byte */
#define LDCN_STATUS_SEND_DIG_IN     (1 << 8)   /* 2 bytes */
#define LDCN_STATUS_SEND_ANA_IN     (1 << 9)   /* 2 bytes */
#define LDCN_STATUS_SEND_WATCHDOG   (1 << 12)  /* 2 bytes */
#define LDCN_STATUS_SEND_MOTOR_POS  (1 << 13)  /* 6 bytes */

/* Load Trajectory Control Byte Bits */
#define LDCN_TRAJ_LOAD_POS          (1 << 0)
#define LDCN_TRAJ_LOAD_VEL          (1 << 1)
#define LDCN_TRAJ_LOAD_ACCEL        (1 << 2)
#define LDCN_TRAJ_LOAD_PWM          (1 << 3)
#define LDCN_TRAJ_SERVO_MODE        (1 << 4)  /* 0=PWM, 1=position */
#define LDCN_TRAJ_PROFILE_MODE      (1 << 5)  /* 0=trapezoid, 1=velocity */
#define LDCN_TRAJ_DIRECTION         (1 << 6)  /* 0=FWD, 1=REV */
#define LDCN_TRAJ_START_NOW         (1 << 7)

/* Stop Motor Control Byte Bits */
#define LDCN_STOP_AMP_ENABLE        (1 << 0)  /* Pic_ae */
#define LDCN_STOP_MOTOR_OFF         (1 << 1)
#define LDCN_STOP_ABRUPT            (1 << 2)
#define LDCN_STOP_SMOOTH            (1 << 3)
#define LDCN_STOP_HERE              (1 << 4)

/* I/O Control Byte Bits */
#define LDCN_IO_BRAKE_MODE          (1 << 0)
#define LDCN_IO_BRAKE_OUT           (1 << 1)
#define LDCN_IO_SET_PATH_FREQ       (1 << 6)

/* Baud Rate Values */
typedef enum {
    LDCN_BAUD_9600    = 0x81,
    LDCN_BAUD_19200   = 0x3F,
    LDCN_BAUD_57600   = 0x14,
    LDCN_BAUD_115200  = 0x0A,
    LDCN_BAUD_125000  = 0x27,
    LDCN_BAUD_312500  = 0x0F,
    LDCN_BAUD_625000  = 0x07,
    LDCN_BAUD_1250000 = 0x03
} ldcn_baud_rate_t;

/* Command Packet Structure */
typedef struct {
    uint8_t header;                          /* Always 0xAA */
    uint8_t address;                         /* Individual or group */
    uint8_t command;                         /* Upper nibble=data bytes, lower=cmd */
    uint8_t data[LDCN_MAX_DATA_BYTES];      /* Command-specific data */
    uint8_t data_len;                        /* Actual data length */
    uint8_t checksum;                        /* Sum of addr+cmd+data */
} __attribute__((packed)) ldcn_cmd_packet_t;

/* Status Packet Structure */
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

/* Packet Construction */
void ldcn_build_command(ldcn_cmd_packet_t *pkt, uint8_t addr, ldcn_cmd_t cmd,
                        const uint8_t *data, uint8_t data_len);
uint8_t ldcn_calc_checksum(const uint8_t *data, size_t len);
bool ldcn_verify_checksum(const ldcn_status_packet_t *pkt);

/* Command Builders */
void ldcn_cmd_hard_reset(ldcn_cmd_packet_t *pkt, uint8_t addr);
void ldcn_cmd_set_address(ldcn_cmd_packet_t *pkt, uint8_t old_addr,
                          uint8_t new_addr, uint8_t group_addr, bool group_leader);
void ldcn_cmd_define_status(ldcn_cmd_packet_t *pkt, uint8_t addr, uint16_t status_bits);
void ldcn_cmd_read_status(ldcn_cmd_packet_t *pkt, uint8_t addr, uint16_t status_bits);
void ldcn_cmd_reset_position(ldcn_cmd_packet_t *pkt, uint8_t addr);
void ldcn_cmd_load_trajectory(ldcn_cmd_packet_t *pkt, uint8_t addr,
                              const ldcn_trajectory_t *traj);
void ldcn_cmd_start_motion(ldcn_cmd_packet_t *pkt, uint8_t addr);
void ldcn_cmd_set_gain(ldcn_cmd_packet_t *pkt, uint8_t addr,
                       const ldcn_gain_params_t *gains);
void ldcn_cmd_stop_motor(ldcn_cmd_packet_t *pkt, uint8_t addr, uint8_t stop_mode);
void ldcn_cmd_set_baud(ldcn_cmd_packet_t *pkt, ldcn_baud_rate_t baud);
void ldcn_cmd_clear_bits(ldcn_cmd_packet_t *pkt, uint8_t addr);
void ldcn_cmd_io_control(ldcn_cmd_packet_t *pkt, uint8_t addr,
                        uint8_t control_byte, uint16_t path_buffer_freq);

/* Status Parsing */
bool ldcn_parse_status(const ldcn_status_packet_t *pkt, ldcn_drive_status_t *status,
                       uint16_t expected_fields);
int ldcn_get_status_data_length(uint16_t status_bits);

/* Utility Functions */
const char *ldcn_cmd_name(ldcn_cmd_t cmd);
const char *ldcn_status_string(const ldcn_drive_status_t *status);
uint32_t ldcn_velocity_to_raw(double counts_per_sec, uint8_t servo_rate_divisor);
uint32_t ldcn_accel_to_raw(double counts_per_sec2, uint8_t servo_rate_divisor);
double ldcn_raw_to_velocity(uint32_t raw_vel, uint8_t servo_rate_divisor);
double ldcn_raw_to_accel(uint32_t raw_accel, uint8_t servo_rate_divisor);

#endif /* LDCN_PROTOCOL_H */
