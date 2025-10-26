/*
 * ldcn_protocol.c
 * 
 * Logosol LDCN Protocol Implementation
 * 
 * Copyright (C) 2024
 * License: GPL-2.0
 */

#include "ldcn_protocol.h"
#include <string.h>
#include <stdio.h>

/* Calculate 8-bit checksum (sum of all bytes) */
uint8_t ldcn_calc_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/* Build a command packet with header, address, command, data, and checksum */
void ldcn_build_command(ldcn_cmd_packet_t *pkt, uint8_t addr, ldcn_cmd_t cmd,
                        const uint8_t *data, uint8_t data_len) {
    if (!pkt) return;
    
    /* Validate data length */
    if (data_len > LDCN_MAX_DATA_BYTES) {
        data_len = LDCN_MAX_DATA_BYTES;
    }
    
    /* Build packet */
    pkt->header = LDCN_HEADER;
    pkt->address = addr;
    pkt->command = ((data_len & 0x0F) << 4) | (cmd & 0x0F);
    pkt->data_len = data_len;
    
    /* Copy data */
    if (data && data_len > 0) {
        memcpy(pkt->data, data, data_len);
    }
    
    /* Calculate checksum: address + command + data bytes */
    uint8_t checksum_data[1 + 1 + LDCN_MAX_DATA_BYTES];
    checksum_data[0] = pkt->address;
    checksum_data[1] = pkt->command;
    if (data_len > 0) {
        memcpy(&checksum_data[2], pkt->data, data_len);
    }
    
    pkt->checksum = ldcn_calc_checksum(checksum_data, 2 + data_len);
}

/* Verify status packet checksum */
bool ldcn_verify_checksum(const ldcn_status_packet_t *pkt) {
    if (!pkt) return false;
    
    uint8_t calc_sum = pkt->status;
    for (int i = 0; i < pkt->data_len; i++) {
        calc_sum += pkt->data[i];
    }
    
    return (calc_sum == pkt->checksum);
}

/* Hard Reset Command */
void ldcn_cmd_hard_reset(ldcn_cmd_packet_t *pkt, uint8_t addr) {
    ldcn_build_command(pkt, addr, LDCN_CMD_HARD_RESET, NULL, 0);
}

/* Set Address Command */
void ldcn_cmd_set_address(ldcn_cmd_packet_t *pkt, uint8_t old_addr,
                          uint8_t new_addr, uint8_t group_addr, bool group_leader) {
    uint8_t data[2];
    
    data[0] = new_addr & 0x7F;  /* Individual address (bit 7 clear) */
    
    /* Group address: bit 7 clear if group leader, set if member */
    if (group_leader) {
        data[1] = group_addr & 0x7F;  /* Clear bit 7 for leader */
    } else {
        data[1] = group_addr | 0x80;  /* Set bit 7 for member */
    }
    
    ldcn_build_command(pkt, old_addr, LDCN_CMD_SET_ADDR, data, 2);
}

/* Define Status Command */
void ldcn_cmd_define_status(ldcn_cmd_packet_t *pkt, uint8_t addr, uint16_t status_bits) {
    uint8_t data[2];
    
    data[0] = status_bits & 0xFF;        /* Low byte */
    data[1] = (status_bits >> 8) & 0xFF; /* High byte */
    
    ldcn_build_command(pkt, addr, LDCN_CMD_DEFINE_STATUS, data, 2);
}

/* Read Status Command (one-time) */
void ldcn_cmd_read_status(ldcn_cmd_packet_t *pkt, uint8_t addr, uint16_t status_bits) {
    uint8_t data[2];
    
    data[0] = status_bits & 0xFF;
    data[1] = (status_bits >> 8) & 0xFF;
    
    ldcn_build_command(pkt, addr, LDCN_CMD_READ_STATUS, data, 2);
}

/* Reset Position Command */
void ldcn_cmd_reset_position(ldcn_cmd_packet_t *pkt, uint8_t addr) {
    ldcn_build_command(pkt, addr, LDCN_CMD_RESET_POS, NULL, 0);
}

/* Load Trajectory Command */
void ldcn_cmd_load_trajectory(ldcn_cmd_packet_t *pkt, uint8_t addr,
                              const ldcn_trajectory_t *traj) {
    uint8_t data[LDCN_MAX_DATA_BYTES];
    uint8_t idx = 0;
    uint8_t control = 0;
    
    /* Build control byte */
    if (traj->servo_mode) control |= LDCN_TRAJ_SERVO_MODE;
    if (traj->velocity_mode) control |= LDCN_TRAJ_PROFILE_MODE;
    if (!traj->direction_fwd) control |= LDCN_TRAJ_DIRECTION;
    if (traj->start_now) control |= LDCN_TRAJ_START_NOW;
    
    /* Add position data (4 bytes, LSB first) */
    if (traj->position != 0 || traj->servo_mode) {
        control |= LDCN_TRAJ_LOAD_POS;
        data[idx++] = (traj->position) & 0xFF;
        data[idx++] = (traj->position >> 8) & 0xFF;
        data[idx++] = (traj->position >> 16) & 0xFF;
        data[idx++] = (traj->position >> 24) & 0xFF;
    }
    
    /* Add velocity data (4 bytes, LSB first) */
    if (traj->velocity != 0) {
        control |= LDCN_TRAJ_LOAD_VEL;
        data[idx++] = (traj->velocity) & 0xFF;
        data[idx++] = (traj->velocity >> 8) & 0xFF;
        data[idx++] = (traj->velocity >> 16) & 0xFF;
        data[idx++] = (traj->velocity >> 24) & 0xFF;
    }
    
    /* Add acceleration data (4 bytes, LSB first) */
    if (traj->acceleration != 0) {
        control |= LDCN_TRAJ_LOAD_ACCEL;
        data[idx++] = (traj->acceleration) & 0xFF;
        data[idx++] = (traj->acceleration >> 8) & 0xFF;
        data[idx++] = (traj->acceleration >> 16) & 0xFF;
        data[idx++] = (traj->acceleration >> 24) & 0xFF;
    }
    
    /* Add PWM value if in PWM mode (2 bytes) */
    if (!traj->servo_mode && traj->pwm != 0) {
        control |= LDCN_TRAJ_LOAD_PWM;
        data[idx++] = traj->pwm & 0xFF;
        data[idx++] = (traj->pwm >> 8) & 0xFF;
    }
    
    /* Shift all data down by 1 to make room for control byte */
    memmove(&data[1], &data[0], idx);
    data[0] = control;
    idx++;
    
    ldcn_build_command(pkt, addr, LDCN_CMD_LOAD_TRAJ, data, idx);
}

/* Start Motion Command */
void ldcn_cmd_start_motion(ldcn_cmd_packet_t *pkt, uint8_t addr) {
    ldcn_build_command(pkt, addr, LDCN_CMD_START_MOTION, NULL, 0);
}

/* Set Gain Parameters Command */
void ldcn_cmd_set_gain(ldcn_cmd_packet_t *pkt, uint8_t addr,
                       const ldcn_gain_params_t *gains) {
    uint8_t data[14];
    
    /* All 16-bit values are LSB first */
    data[0] = gains->kp & 0xFF;
    data[1] = (gains->kp >> 8) & 0xFF;
    data[2] = gains->kd & 0xFF;
    data[3] = (gains->kd >> 8) & 0xFF;
    data[4] = gains->ki & 0xFF;
    data[5] = (gains->ki >> 8) & 0xFF;
    data[6] = gains->il & 0xFF;
    data[7] = (gains->il >> 8) & 0xFF;
    data[8] = gains->ol;
    data[9] = gains->cl;
    data[10] = gains->el & 0xFF;
    data[11] = (gains->el >> 8) & 0xFF;
    data[12] = gains->sr;
    data[13] = gains->db;  /* Reserved, typically 0 */
    
    ldcn_build_command(pkt, addr, LDCN_CMD_SET_GAIN, data, 14);
}

/* Stop Motor Command */
void ldcn_cmd_stop_motor(ldcn_cmd_packet_t *pkt, uint8_t addr, uint8_t stop_mode) {
    ldcn_build_command(pkt, addr, LDCN_CMD_STOP_MOTOR, &stop_mode, 1);
}

/* Set Baud Rate Command (must be sent to group address with no leader) */
void ldcn_cmd_set_baud(ldcn_cmd_packet_t *pkt, ldcn_baud_rate_t baud) {
    uint8_t baud_byte = (uint8_t)baud;
    ldcn_build_command(pkt, LDCN_ADDR_BROADCAST, LDCN_CMD_SET_BAUD,
                      &baud_byte, 1);
}

/* Clear Sticky Bits Command */
void ldcn_cmd_clear_bits(ldcn_cmd_packet_t *pkt, uint8_t addr) {
    ldcn_build_command(pkt, addr, LDCN_CMD_CLEAR_BITS, NULL, 0);
}

/* I/O Control Command (for setting path buffer frequency) */
void ldcn_cmd_io_control(ldcn_cmd_packet_t *pkt, uint8_t addr,
                        uint8_t control_byte, uint16_t path_buffer_freq) {
    uint8_t data[3];
    uint8_t len = 1;
    
    data[0] = control_byte;
    
    /* If setting path buffer frequency, add 2 bytes */
    if (control_byte & LDCN_IO_SET_PATH_FREQ) {
        data[1] = path_buffer_freq & 0xFF;
        data[2] = (path_buffer_freq >> 8) & 0xFF;
        len = 3;
    }
    
    ldcn_build_command(pkt, addr, LDCN_CMD_IO_CONTROL, data, len);
}

/* Calculate expected status data length based on requested fields */
int ldcn_get_status_data_length(uint16_t status_bits) {
    int len = 0;
    
    if (status_bits & LDCN_STATUS_SEND_POS) len += 4;
    if (status_bits & LDCN_STATUS_SEND_AD) len += 1;
    if (status_bits & LDCN_STATUS_SEND_VEL) len += 2;
    if (status_bits & LDCN_STATUS_SEND_AUX) len += 1;
    if (status_bits & LDCN_STATUS_SEND_HOME) len += 4;
    if (status_bits & LDCN_STATUS_SEND_ID_VER) len += 2;
    if (status_bits & LDCN_STATUS_SEND_POS_ERR) len += 2;
    if (status_bits & LDCN_STATUS_SEND_BUFFER_CNT) len += 1;
    if (status_bits & LDCN_STATUS_SEND_DIG_IN) len += 2;
    if (status_bits & LDCN_STATUS_SEND_ANA_IN) len += 2;
    if (status_bits & LDCN_STATUS_SEND_WATCHDOG) len += 2;
    if (status_bits & LDCN_STATUS_SEND_MOTOR_POS) len += 6;
    
    return len;
}

/* Parse status packet into structured data */
bool ldcn_parse_status(const ldcn_status_packet_t *pkt, ldcn_drive_status_t *status,
                       uint16_t expected_fields) {
    if (!pkt || !status) return false;
    
    /* Verify checksum */
    if (!ldcn_verify_checksum(pkt)) {
        return false;
    }
    
    /* Parse status byte */
    status->move_done = (pkt->status & LDCN_STATUS_MOVE_DONE) != 0;
    status->checksum_error = (pkt->status & LDCN_STATUS_CKSUM_ERROR) != 0;
    status->current_limit = (pkt->status & LDCN_STATUS_CURRENT_LIMIT) != 0;
    status->power_on = (pkt->status & LDCN_STATUS_POWER_ON) != 0;
    status->position_error = (pkt->status & LDCN_STATUS_POS_ERROR) != 0;
    status->home_in_progress = (pkt->status & LDCN_STATUS_HOME_IN_PROG) != 0;
    
    /* Parse optional data fields in order */
    int idx = 0;
    
    if (expected_fields & LDCN_STATUS_SEND_POS) {
        status->position = pkt->data[idx] |
                          (pkt->data[idx+1] << 8) |
                          (pkt->data[idx+2] << 16) |
                          (pkt->data[idx+3] << 24);
        idx += 4;
    }
    
    if (expected_fields & LDCN_STATUS_SEND_AD) {
        status->ad_value = pkt->data[idx++];
    }
    
    if (expected_fields & LDCN_STATUS_SEND_VEL) {
        status->velocity = pkt->data[idx] | (pkt->data[idx+1] << 8);
        idx += 2;
    }
    
    if (expected_fields & LDCN_STATUS_SEND_AUX) {
        uint8_t aux = pkt->data[idx++];
        status->servo_on = (aux & LDCN_AUX_STATUS_SERVO_ON) != 0;
        status->path_mode = (aux & LDCN_AUX_STATUS_PATH_MODE) != 0;
        status->servo_overrun = (aux & LDCN_AUX_STATUS_SERVO_OVERRUN) != 0;
    }
    
    if (expected_fields & LDCN_STATUS_SEND_HOME) {
        status->home_position = pkt->data[idx] |
                               (pkt->data[idx+1] << 8) |
                               (pkt->data[idx+2] << 16) |
                               (pkt->data[idx+3] << 24);
        idx += 4;
    }
    
    if (expected_fields & LDCN_STATUS_SEND_ID_VER) {
        status->device_id = pkt->data[idx++];
        status->version = pkt->data[idx++];
    }
    
    if (expected_fields & LDCN_STATUS_SEND_POS_ERR) {
        status->following_error = pkt->data[idx] | (pkt->data[idx+1] << 8);
        idx += 2;
    }
    
    if (expected_fields & LDCN_STATUS_SEND_BUFFER_CNT) {
        status->buffer_count = pkt->data[idx++];
    }
    
    return true;
}

/* Velocity conversion: counts/sec to raw format (counts/tick * 65536) */
uint32_t ldcn_velocity_to_raw(double counts_per_sec, uint8_t servo_rate_divisor) {
    /* Servo tick time in seconds */
    double tick_time = 51.2e-6 * servo_rate_divisor;
    
    /* Counts per tick */
    double counts_per_tick = counts_per_sec * tick_time;
    
    /* Convert to fixed-point with 16-bit fractional part */
    return (uint32_t)(counts_per_tick * 65536.0);
}

/* Acceleration conversion: counts/sec² to raw format (counts/tick² * 65536) */
uint32_t ldcn_accel_to_raw(double counts_per_sec2, uint8_t servo_rate_divisor) {
    /* Servo tick time in seconds */
    double tick_time = 51.2e-6 * servo_rate_divisor;
    
    /* Counts per tick per tick */
    double counts_per_tick2 = counts_per_sec2 * tick_time * tick_time;
    
    /* Convert to fixed-point with 16-bit fractional part */
    return (uint32_t)(counts_per_tick2 * 65536.0);
}

/* Raw velocity to counts/sec */
double ldcn_raw_to_velocity(uint32_t raw_vel, uint8_t servo_rate_divisor) {
    double tick_time = 51.2e-6 * servo_rate_divisor;
    double counts_per_tick = (double)raw_vel / 65536.0;
    return counts_per_tick / tick_time;
}

/* Raw acceleration to counts/sec² */
double ldcn_raw_to_accel(uint32_t raw_accel, uint8_t servo_rate_divisor) {
    double tick_time = 51.2e-6 * servo_rate_divisor;
    double counts_per_tick2 = (double)raw_accel / 65536.0;
    return counts_per_tick2 / (tick_time * tick_time);
}

/* Get command name string */
const char *ldcn_cmd_name(ldcn_cmd_t cmd) {
    switch (cmd) {
        case LDCN_CMD_RESET_POS: return "RESET_POS";
        case LDCN_CMD_SET_ADDR: return "SET_ADDR";
        case LDCN_CMD_DEFINE_STATUS: return "DEFINE_STATUS";
        case LDCN_CMD_READ_STATUS: return "READ_STATUS";
        case LDCN_CMD_LOAD_TRAJ: return "LOAD_TRAJ";
        case LDCN_CMD_START_MOTION: return "START_MOTION";
        case LDCN_CMD_SET_GAIN: return "SET_GAIN";
        case LDCN_CMD_STOP_MOTOR: return "STOP_MOTOR";
        case LDCN_CMD_IO_CONTROL: return "IO_CONTROL";
        case LDCN_CMD_SET_HOME_MODE: return "SET_HOME_MODE";
        case LDCN_CMD_SET_BAUD: return "SET_BAUD";
        case LDCN_CMD_CLEAR_BITS: return "CLEAR_BITS";
        case LDCN_CMD_SAVE_HOME: return "SAVE_HOME";
        case LDCN_CMD_ADD_PATH_PTS: return "ADD_PATH_PTS";
        case LDCN_CMD_NOP: return "NOP";
        case LDCN_CMD_HARD_RESET: return "HARD_RESET";
        default: return "UNKNOWN";
    }
}

/* Get status string (for debugging) */
const char *ldcn_status_string(const ldcn_drive_status_t *status) {
    static char buf[256];
    
    snprintf(buf, sizeof(buf),
             "pos=%d vel=%d err=%d %s%s%s%s%s",
             status->position,
             status->velocity,
             status->following_error,
             status->servo_on ? "SERVO " : "",
             status->move_done ? "DONE " : "",
             status->position_error ? "POSERR " : "",
             status->current_limit ? "ILIMIT " : "",
             status->home_in_progress ? "HOMING " : "");
    
    return buf;
}
