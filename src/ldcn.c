/*
 * ldcn.c
 * 
 * LinuxCNC HAL Userspace Component for Logosol LDCN Servo Drives
 * 
 * Non-realtime driver for LS-231SE and LS-2310g2 servo drives
 * 
 * Usage: loadusr -W ldcn port=/dev/ttyUSB0 baud=115200 axes=3
 * 
 * Copyright (C) 2024
 * License: GPL-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>

#include "hal.h"
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"

#include "ldcn_protocol.h"
#include "ldcn_serial.h"

/* Component name */
#define COMP_NAME "ldcn"

/* Default values */
#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD 115200
#define DEFAULT_AXES 3
#define DEFAULT_SERVO_RATE 20  /* ~1ms update rate */
#define DEFAULT_PATH_BUFFER_FREQ 100  /* 5.12ms per point */

/* Per-Axis HAL Data */
typedef struct {
    /* Input Pins (LinuxCNC → Driver) */
    hal_bit_t *enable;
    hal_float_t *pos_cmd;
    hal_float_t *vel_cmd;
    hal_float_t *accel;
    hal_bit_t *reset_pos;
    
    /* Output Pins (Driver → LinuxCNC) */
    hal_float_t *pos_fb;
    hal_float_t *vel_fb;
    hal_float_t *ferror;
    hal_bit_t *is_homed;
    hal_bit_t *fault;
    hal_bit_t *amp_enabled;
    
    /* Parameters */
    hal_float_t scale;
    hal_u32_t kp;
    hal_u32_t kd;
    hal_u32_t ki;
    hal_u32_t address;
    
    /* Internal state */
    uint8_t ldcn_addr;
    bool enabled_prev;
    bool initialized;
    ldcn_drive_status_t status;
    
} axis_data_t;

/* Global HAL Data */
typedef struct {
    int num_axes;
    axis_data_t *axes;
    ldcn_serial_port_t *port;
    
    /* Configuration */
    char port_name[256];
    int baud_rate;
    uint8_t servo_rate_divisor;
    bool path_mode;
    uint16_t path_buffer_freq;
    
    /* Runtime state */
    bool running;
    int comp_id;
    
} hal_data_t;

static hal_data_t *hal_data = NULL;
static volatile sig_atomic_t done = 0;

/* Signal handler */
static void quit(int sig) {
    done = 1;
}

/* Initialize a single drive */
static int init_drive(int axis_num) {
    axis_data_t *axis = &hal_data->axes[axis_num];
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    ldcn_gain_params_t gains;
    ldcn_trajectory_t traj;
    int ret;
    
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Initializing axis %d (addr 0x%02X)\n",
                   COMP_NAME, axis_num, axis->ldcn_addr);
    
    /* Step 1: Set address */
    ldcn_cmd_set_address(&cmd, LDCN_ADDR_DEFAULT,
                        axis->ldcn_addr, LDCN_DEFAULT_GROUP_ADDR, false);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: Failed to set address for axis %d\n",
                       COMP_NAME, axis_num);
        return -1;
    }
    
    usleep(10000);  /* 10ms delay */
    
    /* Step 2: Configure status reporting */
    uint16_t status_bits = LDCN_STATUS_SEND_POS | LDCN_STATUS_SEND_VEL |
                          LDCN_STATUS_SEND_AUX | LDCN_STATUS_SEND_POS_ERR;
    ldcn_cmd_define_status(&cmd, axis->ldcn_addr, status_bits);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                       "%s: Failed to set status format for axis %d\n",
                       COMP_NAME, axis_num);
    }
    
    /* Step 3: Set PID gains */
    memset(&gains, 0, sizeof(gains));
    gains.kp = axis->kp ? axis->kp : 50;     /* Default gains */
    gains.kd = axis->kd ? axis->kd : 8000;
    gains.ki = axis->ki ? axis->ki : 50;
    gains.il = 500;
    gains.ol = 0xFA;  /* Output limit */
    gains.cl = 0;     /* Current limit (0 = disabled) */
    gains.el = 1024;  /* Position error limit */
    gains.sr = hal_data->servo_rate_divisor;
    gains.db = 0;
    
    ldcn_cmd_set_gain(&cmd, axis->ldcn_addr, &gains);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: Failed to set gains for axis %d\n",
                       COMP_NAME, axis_num);
        return -1;
    }
    
    /* Step 4: Initialize trajectory (required for internal registers) */
    memset(&traj, 0, sizeof(traj));
    traj.position = 0;
    traj.velocity = 0;
    traj.acceleration = 100;  /* Must be non-zero */
    traj.servo_mode = true;
    traj.velocity_mode = false;
    traj.start_now = false;
    
    ldcn_cmd_load_trajectory(&cmd, axis->ldcn_addr, &traj);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: Failed to load initial trajectory for axis %d\n",
                       COMP_NAME, axis_num);
        return -1;
    }
    
    /* Step 5: Enable servo (stop motor with amp enable + stop abrupt) */
    ldcn_cmd_stop_motor(&cmd, axis->ldcn_addr,
                       LDCN_STOP_AMP_ENABLE | LDCN_STOP_ABRUPT);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: Failed to enable servo for axis %d\n",
                       COMP_NAME, axis_num);
        return -1;
    }
    
    axis->initialized = true;
    
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: Axis %d initialized successfully\n",
                   COMP_NAME, axis_num);
    
    return 0;
}

/* Update a single axis */
static void update_axis(int axis_num) {
    axis_data_t *axis = &hal_data->axes[axis_num];
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status_pkt;
    ldcn_trajectory_t traj;
    int ret;
    
    if (!axis->initialized) return;
    
    /* Handle enable/disable transitions */
    if (*axis->enable && !axis->enabled_prev) {
        /* Enable transition */
        ldcn_cmd_stop_motor(&cmd, axis->ldcn_addr,
                           LDCN_STOP_AMP_ENABLE | LDCN_STOP_ABRUPT);
        ldcn_serial_send_command(hal_data->port, &cmd);
        axis->enabled_prev = true;
    } else if (!*axis->enable && axis->enabled_prev) {
        /* Disable transition */
        ldcn_cmd_stop_motor(&cmd, axis->ldcn_addr, LDCN_STOP_MOTOR_OFF);
        ldcn_serial_send_command(hal_data->port, &cmd);
        axis->enabled_prev = false;
    }
    
    /* Handle position reset */
    if (*axis->reset_pos) {
        ldcn_cmd_reset_position(&cmd, axis->ldcn_addr);
        ldcn_serial_send_command(hal_data->port, &cmd);
        *axis->reset_pos = 0;  /* Clear the bit */
    }
    
    /* Send position command if enabled */
    if (*axis->enable) {
        /* Convert from machine units to encoder counts */
        int32_t pos_counts = (int32_t)(*axis->pos_cmd * axis->scale);
        
        /* Convert velocity and acceleration */
        uint32_t vel_raw = ldcn_velocity_to_raw(*axis->vel_cmd * axis->scale,
                                                hal_data->servo_rate_divisor);
        uint32_t accel_raw = ldcn_accel_to_raw(*axis->accel * axis->scale,
                                              hal_data->servo_rate_divisor);
        
        /* Build trajectory */
        memset(&traj, 0, sizeof(traj));
        traj.position = pos_counts;
        traj.velocity = vel_raw;
        traj.acceleration = accel_raw ? accel_raw : 100;
        traj.servo_mode = true;
        traj.velocity_mode = false;  /* Use trapezoidal profile */
        traj.start_now = true;
        
        ldcn_cmd_load_trajectory(&cmd, axis->ldcn_addr, &traj);
        ldcn_serial_send_command(hal_data->port, &cmd);
    }
    
    /* Read status */
    uint16_t status_bits = LDCN_STATUS_SEND_POS | LDCN_STATUS_SEND_VEL |
                          LDCN_STATUS_SEND_AUX | LDCN_STATUS_SEND_POS_ERR;
    ldcn_cmd_read_status(&cmd, axis->ldcn_addr, status_bits);
    
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status_pkt, 50);
    if (ret > 0) {
        /* Parse status */
        if (ldcn_parse_status(&status_pkt, &axis->status, status_bits)) {
            /* Update HAL pins with feedback */
            *axis->pos_fb = (hal_float_t)axis->status.position / axis->scale;
            *axis->vel_fb = ldcn_raw_to_velocity(axis->status.velocity,
                                                hal_data->servo_rate_divisor) / axis->scale;
            *axis->ferror = (hal_float_t)axis->status.following_error / axis->scale;
            *axis->amp_enabled = axis->status.servo_on;
            *axis->fault = axis->status.position_error || axis->status.current_limit;
        }
    }
}

/* Main update loop */
static void update_all(void) {
    for (int i = 0; i < hal_data->num_axes; i++) {
        update_axis(i);
    }
}

/* Export HAL pins and parameters for one axis */
static int export_axis(int num) {
    axis_data_t *axis = &hal_data->axes[num];
    char name[HAL_NAME_LEN + 1];
    int ret;
    
    /* Input pins */
    ret = hal_pin_bit_newf(HAL_IN, &axis->enable, hal_data->comp_id,
                          "%s.%d.enable", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_float_newf(HAL_IN, &axis->pos_cmd, hal_data->comp_id,
                            "%s.%d.position-cmd", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_float_newf(HAL_IN, &axis->vel_cmd, hal_data->comp_id,
                            "%s.%d.velocity-cmd", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_float_newf(HAL_IN, &axis->accel, hal_data->comp_id,
                            "%s.%d.acceleration", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_bit_newf(HAL_IN, &axis->reset_pos, hal_data->comp_id,
                          "%s.%d.reset-position", COMP_NAME, num);
    if (ret != 0) return ret;
    
    /* Output pins */
    ret = hal_pin_float_newf(HAL_OUT, &axis->pos_fb, hal_data->comp_id,
                            "%s.%d.position-fb", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_float_newf(HAL_OUT, &axis->vel_fb, hal_data->comp_id,
                            "%s.%d.velocity-fb", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_float_newf(HAL_OUT, &axis->ferror, hal_data->comp_id,
                            "%s.%d.following-error", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_bit_newf(HAL_OUT, &axis->is_homed, hal_data->comp_id,
                          "%s.%d.is-homed", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_bit_newf(HAL_OUT, &axis->fault, hal_data->comp_id,
                          "%s.%d.fault", COMP_NAME, num);
    if (ret != 0) return ret;
    
    ret = hal_pin_bit_newf(HAL_OUT, &axis->amp_enabled, hal_data->comp_id,
                          "%s.%d.amp-enabled", COMP_NAME, num);
    if (ret != 0) return ret;
    
    /* Parameters */
    ret = hal_param_float_newf(HAL_RW, &axis->scale, hal_data->comp_id,
                              "%s.%d.scale", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->scale = 1.0;  /* Default scale */
    
    ret = hal_param_u32_newf(HAL_RW, &axis->kp, hal_data->comp_id,
                            "%s.%d.kp", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->kp = 50;
    
    ret = hal_param_u32_newf(HAL_RW, &axis->kd, hal_data->comp_id,
                            "%s.%d.kd", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->kd = 8000;
    
    ret = hal_param_u32_newf(HAL_RW, &axis->ki, hal_data->comp_id,
                            "%s.%d.ki", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->ki = 50;
    
    ret = hal_param_u32_newf(HAL_RO, &axis->address, hal_data->comp_id,
                            "%s.%d.address", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->address = axis->ldcn_addr;
    
    return 0;
}

/* Print usage */
static void usage(void) {
    printf("Usage: ldcn [options]\n");
    printf("Options:\n");
    printf("  port=<device>        Serial port (default: %s)\n", DEFAULT_PORT);
    printf("  baud=<rate>          Baud rate (default: %d)\n", DEFAULT_BAUD);
    printf("  axes=<count>         Number of axes (default: %d)\n", DEFAULT_AXES);
    printf("  servo_rate=<divisor> Servo rate divisor (default: %d)\n", DEFAULT_SERVO_RATE);
    printf("  path_mode=<0|1>      Enable path mode (default: 0)\n");
    printf("  path_buffer_freq=<n> Path buffer frequency (default: %d)\n", DEFAULT_PATH_BUFFER_FREQ);
}

/* Parse command line arguments */
static int parse_args(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "port=", 5) == 0) {
            strncpy(hal_data->port_name, &argv[i][5], sizeof(hal_data->port_name) - 1);
        } else if (strncmp(argv[i], "baud=", 5) == 0) {
            hal_data->baud_rate = atoi(&argv[i][5]);
        } else if (strncmp(argv[i], "axes=", 5) == 0) {
            hal_data->num_axes = atoi(&argv[i][5]);
            if (hal_data->num_axes < 1 || hal_data->num_axes > LDCN_MAX_AXES) {
                fprintf(stderr, "Error: axes must be between 1 and %d\n", LDCN_MAX_AXES);
                return -1;
            }
        } else if (strncmp(argv[i], "servo_rate=", 11) == 0) {
            hal_data->servo_rate_divisor = atoi(&argv[i][11]);
        } else if (strncmp(argv[i], "path_mode=", 10) == 0) {
            hal_data->path_mode = atoi(&argv[i][10]) != 0;
        } else if (strncmp(argv[i], "path_buffer_freq=", 17) == 0) {
            hal_data->path_buffer_freq = atoi(&argv[i][17]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            usage();
            return -1;
        }
    }
    
    return 0;
}

int main(int argc, char **argv) {
    int ret;
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    
    /* Allocate HAL data structure */
    hal_data = calloc(1, sizeof(hal_data_t));
    if (!hal_data) {
        fprintf(stderr, "%s: ERROR: Failed to allocate memory\n", COMP_NAME);
        return -1;
    }
    
    /* Set defaults */
    strncpy(hal_data->port_name, DEFAULT_PORT, sizeof(hal_data->port_name) - 1);
    hal_data->baud_rate = DEFAULT_BAUD;
    hal_data->num_axes = DEFAULT_AXES;
    hal_data->servo_rate_divisor = DEFAULT_SERVO_RATE;
    hal_data->path_mode = false;
    hal_data->path_buffer_freq = DEFAULT_PATH_BUFFER_FREQ;
    
    /* Parse command line */
    if (parse_args(argc, argv) < 0) {
        free(hal_data);
        return -1;
    }
    
    /* Initialize HAL */
    hal_data->comp_id = hal_init(COMP_NAME);
    if (hal_data->comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", COMP_NAME);
        free(hal_data);
        return -1;
    }
    
    /* Allocate axis data */
    hal_data->axes = calloc(hal_data->num_axes, sizeof(axis_data_t));
    if (!hal_data->axes) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Failed to allocate axis data\n", COMP_NAME);
        hal_exit(hal_data->comp_id);
        free(hal_data);
        return -1;
    }
    
    /* Assign LDCN addresses */
    for (int i = 0; i < hal_data->num_axes; i++) {
        hal_data->axes[i].ldcn_addr = i + 1;  /* Addresses 1-N */
    }
    
    /* Export HAL pins/parameters */
    for (int i = 0; i < hal_data->num_axes; i++) {
        ret = export_axis(i);
        if (ret != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                          "%s: ERROR: Failed to export axis %d\n",
                          COMP_NAME, i);
            hal_exit(hal_data->comp_id);
            free(hal_data->axes);
            free(hal_data);
            return -1;
        }
    }
    
    /* Ready */
    hal_ready(hal_data->comp_id);
    
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: Initialized with %d axes on %s at %d baud\n",
                   COMP_NAME, hal_data->num_axes,
                   hal_data->port_name, hal_data->baud_rate);
    
    /*
     * LDCN Initialization Sequence:
     * 1. Open port at 19200 baud (LDCN protocol default)
     * 2. Reset all drives to establish known state
     * 3. Initialize each drive with addressing and parameters
     * 4. Upgrade to higher baud rate if configured
     */

    /* Step 1: Open serial port at 19200 (LDCN default) */
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: Opening %s at 19200 baud (LDCN default)\n",
                   COMP_NAME, hal_data->port_name);
    hal_data->port = ldcn_serial_open(hal_data->port_name, 19200);
    if (!hal_data->port) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: ERROR: Failed to open serial port %s\n",
                       COMP_NAME, hal_data->port_name);
        hal_exit(hal_data->comp_id);
        free(hal_data->axes);
        free(hal_data);
        return -1;
    }

    /* Step 2: Reset all drives */
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Resetting all LDCN devices...\n", COMP_NAME);
    ldcn_cmd_hard_reset(&cmd, LDCN_ADDR_BROADCAST);
    ldcn_serial_send_command(hal_data->port, &cmd);
    usleep(100000);  /* 100ms delay after reset */

    /* Step 3: Initialize each drive at 19200 baud */
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Initializing %d drives at 19200 baud...\n",
                   COMP_NAME, hal_data->num_axes);
    for (int i = 0; i < hal_data->num_axes; i++) {
        if (init_drive(i) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                          "%s: ERROR: Failed to initialize axis %d\n",
                          COMP_NAME, i);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            free(hal_data->axes);
            free(hal_data);
            return -1;
        }
    }

    /* Step 4: Upgrade baud rate if different from 19200 */
    if (hal_data->baud_rate != 19200) {
        ldcn_baud_rate_t baud_code;

        /* Map baud rate to LDCN protocol value */
        switch (hal_data->baud_rate) {
            case 57600:   baud_code = LDCN_BAUD_57600; break;
            case 115200:  baud_code = LDCN_BAUD_115200; break;
            case 125000:  baud_code = LDCN_BAUD_125000; break;
            case 312500:  baud_code = LDCN_BAUD_312500; break;
            case 625000:  baud_code = LDCN_BAUD_625000; break;
            case 1250000: baud_code = LDCN_BAUD_1250000; break;
            default:
                rtapi_print_msg(RTAPI_MSG_WARN,
                              "%s: Unsupported baud rate %d, staying at 19200\n",
                              COMP_NAME, hal_data->baud_rate);
                hal_data->baud_rate = 19200;
                goto skip_baud_upgrade;
        }

        rtapi_print_msg(RTAPI_MSG_INFO,
                       "%s: Upgrading communication speed to %d baud...\n",
                       COMP_NAME, hal_data->baud_rate);

        /* Send baud rate change command to all drives */
        ldcn_cmd_set_baud(&cmd, baud_code);
        ldcn_serial_send_command(hal_data->port, &cmd);

        /* Wait for drives to switch baud rate */
        usleep(50000);  /* 50ms delay */

        /* Change host serial port baud rate */
        ret = ldcn_serial_set_baud(hal_data->port, hal_data->baud_rate);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                          "%s: ERROR: Failed to change host baud rate to %d\n",
                          COMP_NAME, hal_data->baud_rate);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            free(hal_data->axes);
            free(hal_data);
            return -1;
        }

        /* Verify communication at new baud rate */
        rtapi_print_msg(RTAPI_MSG_INFO,
                       "%s: Verifying communication at %d baud...\n",
                       COMP_NAME, hal_data->baud_rate);
        ldcn_cmd_read_status(&cmd, hal_data->axes[0].ldcn_addr,
                            LDCN_STATUS_SEND_POS);
        ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                          "%s: ERROR: Communication failed at %d baud\n",
                          COMP_NAME, hal_data->baud_rate);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            free(hal_data->axes);
            free(hal_data);
            return -1;
        }

        rtapi_print_msg(RTAPI_MSG_INFO,
                       "%s: Successfully upgraded to %d baud\n",
                       COMP_NAME, hal_data->baud_rate);
    }

skip_baud_upgrade:
    
    /* Setup signal handlers */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    
    /* Main loop - run at ~1ms (1000 Hz) */
    hal_data->running = true;
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Entering main loop\n", COMP_NAME);
    
    while (!done) {
        update_all();
        usleep(1000);  /* 1ms update rate */
    }
    
    /* Cleanup */
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Shutting down\n", COMP_NAME);
    
    /* Disable all drives */
    for (int i = 0; i < hal_data->num_axes; i++) {
        ldcn_cmd_stop_motor(&cmd, hal_data->axes[i].ldcn_addr, LDCN_STOP_MOTOR_OFF);
        ldcn_serial_send_command(hal_data->port, &cmd);
    }
    
    ldcn_serial_close(hal_data->port);
    hal_exit(hal_data->comp_id);
    free(hal_data->axes);
    free(hal_data);
    
    return 0;
}
