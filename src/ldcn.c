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
#include "rtapi_string.h"

#include "ldcn_protocol.h"
#include "ldcn_serial.h"

/* Component name */
#define COMP_NAME "ldcn"

/* Default values */
#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD 125000  /* Target baud rate */
#define DEFAULT_AXES 3
#define DEFAULT_SERVO_RATE 20  /* ~1ms update rate */
#define DEFAULT_PATH_BUFFER_FREQ 100  /* 5.12ms per point */
#define SUPERVISOR_ADDR 6  /* LS-2310g2 supervisor address */
#define TOTAL_LDCN_DEVICES 6  /* 5 servos + 1 supervisor */

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
    
    /* Parameters (stored in HAL shared memory, not local pointers) */
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
    int debug_level;            /* 0=errors only, 1=normal, 2=verbose TX/RX */
    uint8_t servo_rate_divisor;
    bool path_mode;
    uint16_t path_buffer_freq;

    /* Initialization phase control */
    bool do_detect_baud;
    bool do_reset;
    bool do_discover;
    bool do_assign;
    bool do_upgrade_baud;
    bool do_configure_supervisor;
    bool do_wait_power;
    bool do_init_servos;
    bool do_full_init;  /* Run all phases */

    /* E-stop monitoring */
    hal_bit_t *estop_ok;        /* OUT: TRUE when system is NOT in E-stop */
    hal_u32_t *din_raw;         /* OUT: Raw digital input word from supervisor */
    hal_bit_t *homing_active;   /* OUT: TRUE when any axis is homing */

    /* Runtime state */
    bool running;
    bool power_up_needed;           /* TRUE until first F2 triggers power-up sequence */
    bool power_up_in_progress;      /* TRUE while waiting for power button */
    bool waiting_for_power_button;  /* TRUE while polling for power button press */
    uint8_t power_button_baseline;  /* Baseline status before power button press */
    int comp_id;

} hal_data_t;

static hal_data_t *hal_data = NULL;
static volatile sig_atomic_t done = 0;

/* Signal handler */
static void quit(int sig) {
    done = 1;
}

/*
 * LDCN Network Initialization Functions
 *
 * Based on working Python implementation (ldcn_initialization_test.py)
 * Sequence: auto-detect → reset → address → upgrade → supervisor → power → servos
 */

/* Try to communicate at a specific baud rate */
static int try_baud_rate(const char *port_name, int baud) {
    ldcn_serial_port_t *port;
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    int ret;

    port = ldcn_serial_open(port_name, baud);
    if (!port) {
        return -1;
    }

    usleep(200000);  /* 200ms settling time */

    /* Try NOP command on a few common addresses */
    int test_addrs[] = {1, 2, 3, 6};
    for (int i = 0; i < 4; i++) {
        ldcn_cmd_packet_t nop_cmd;
        ldcn_build_command(&nop_cmd, test_addrs[i], LDCN_CMD_NOP, NULL, 0);
        ret = ldcn_serial_exchange(port, &nop_cmd, &status, 100);
        if (ret >= 0) {
            rtapi_print_msg(RTAPI_MSG_INFO,
                          "%s: Device %d responded at %d baud\n",
                          COMP_NAME, test_addrs[i], baud);
            ldcn_serial_close(port);
            return baud;
        }
    }

    ldcn_serial_close(port);
    return -1;
}

/* Auto-detect current baud rate */
static int detect_baud_rate(const char *port_name) {
    int baud_rates[] = {19200, 125000, 115200, 57600, 9600, 38400};
    int num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Auto-detecting current baud rate...\n", COMP_NAME);
    fprintf(stderr, "%s: Auto-detecting baud rate...\n", COMP_NAME);
    fflush(stderr);

    for (int i = 0; i < num_rates; i++) {
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Trying %d baud...\n",
                       COMP_NAME, baud_rates[i]);
        fprintf(stderr, "%s: Trying %d baud...\n", COMP_NAME, baud_rates[i]);
        fflush(stderr);
        if (try_baud_rate(port_name, baud_rates[i]) >= 0) {
            rtapi_print_msg(RTAPI_MSG_INFO,
                          "%s: Found devices at %d baud\n",
                          COMP_NAME, baud_rates[i]);
            fprintf(stderr, "%s: Found devices at %d baud\n", COMP_NAME, baud_rates[i]);
            fflush(stderr);
            return baud_rates[i];
        }
    }

    rtapi_print_msg(RTAPI_MSG_WARN,
                   "%s: No devices found at any common baud rate, assuming 19200\n",
                   COMP_NAME);
    fprintf(stderr, "%s: No devices found, assuming 19200 baud\n", COMP_NAME);
    fflush(stderr);
    return 19200;
}

/* Hard reset all LDCN devices */
static int hard_reset_network(ldcn_serial_port_t *port) {
    ldcn_cmd_packet_t cmd;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Hard reset all LDCN devices...\n", COMP_NAME);
    fprintf(stderr, "%s: Sending hard reset to all devices...\n", COMP_NAME);
    fflush(stderr);

    ldcn_cmd_hard_reset(&cmd, LDCN_ADDR_BROADCAST);
    ldcn_serial_send_command(port, &cmd);

    usleep(2000000);  /* 2 second delay for reset to complete */

    fprintf(stderr, "%s: Reset complete (devices at 19200 baud)\n", COMP_NAME);
    fflush(stderr);

    return 0;
}

/* Query and discover devices on the network */
static int discover_devices(ldcn_serial_port_t *port, int max_addr) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    int found = 0;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Discovering devices (addresses 1-%d)...\n",
                   COMP_NAME, max_addr);

    for (int addr = 1; addr <= max_addr; addr++) {
        ldcn_build_command(&cmd, addr, LDCN_CMD_NOP, NULL, 0);
        int ret = ldcn_serial_exchange(port, &cmd, &status, 100);
        if (ret >= 0) {
            rtapi_print_msg(RTAPI_MSG_INFO, "%s: Found device at address %d\n",
                          COMP_NAME, addr);
            found++;
        }
        usleep(100000);  /* 100ms between queries */
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Found %d device(s)\n", COMP_NAME, found);
    return found;
}

/* Assign addresses to all LDCN devices */
static int assign_device_addresses(ldcn_serial_port_t *port, int num_devices) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Assigning addresses to %d devices...\n",
                   COMP_NAME, num_devices);
    fprintf(stderr, "%s: Assigning addresses to %d devices...\n", COMP_NAME, num_devices);
    fflush(stderr);

    for (int i = 1; i <= num_devices; i++) {
        /* SET_ADDRESS command: old_addr=0x00, new_addr=i, group=0xFF */
        ldcn_cmd_set_address(&cmd, LDCN_ADDR_DEFAULT, i, 0xFF, false);
        int ret = ldcn_serial_exchange(port, &cmd, &status, 100);

        if (ret >= 0) {
            rtapi_print_msg(RTAPI_MSG_INFO, "%s: Device %d: ✓\n", COMP_NAME, i);
            fprintf(stderr, "%s: Device %d addressed\n", COMP_NAME, i);
            fflush(stderr);
        } else {
            rtapi_print_msg(RTAPI_MSG_WARN, "%s: Device %d: ✗\n", COMP_NAME, i);
        }

        usleep(300000);  /* 300ms between addressing */
    }

    usleep(2000000);  /* 2 second settling time after addressing */
    return 0;
}

/* Verify all devices respond at current baud rate */
static int verify_devices(ldcn_serial_port_t *port, int num_devices, int baud) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    int all_ok = 1;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Verifying %d devices at %d baud...\n",
                   COMP_NAME, num_devices, baud);

    for (int addr = 1; addr <= num_devices; addr++) {
        ldcn_build_command(&cmd, addr, LDCN_CMD_NOP, NULL, 0);
        int ret = ldcn_serial_exchange(port, &cmd, &status, 100);

        if (ret >= 0) {
            rtapi_print_msg(RTAPI_MSG_INFO, "%s: Device %d: ✓\n", COMP_NAME, addr);
        } else {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: Device %d: ✗\n", COMP_NAME, addr);
            all_ok = 0;
        }
    }

    if (!all_ok) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: Not all devices responding at %d baud\n",
                       COMP_NAME, baud);
        return -1;
    }

    return 0;
}

/* Upgrade communication speed to target baud rate */
static int upgrade_baud_rate(ldcn_serial_port_t **port_ptr, const char *port_name,
                             int from_baud, int to_baud) {
    ldcn_cmd_packet_t cmd;
    ldcn_baud_rate_t baud_code;
    ldcn_serial_port_t *old_port = *port_ptr;
    ldcn_serial_port_t *new_port;

    if (from_baud == to_baud) {
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Already at %d baud, skipping upgrade\n",
                       COMP_NAME, to_baud);
        return 0;
    }

    /* Map baud rate to LDCN protocol value */
    switch (to_baud) {
        case 57600:   baud_code = LDCN_BAUD_57600; break;
        case 115200:  baud_code = LDCN_BAUD_115200; break;
        case 125000:  baud_code = LDCN_BAUD_125000; break;
        case 312500:  baud_code = LDCN_BAUD_312500; break;
        case 625000:  baud_code = LDCN_BAUD_625000; break;
        case 1250000: baud_code = LDCN_BAUD_1250000; break;
        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: Unsupported baud rate %d\n",
                          COMP_NAME, to_baud);
            return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Upgrading from %d to %d baud...\n",
                   COMP_NAME, from_baud, to_baud);
    fprintf(stderr, "%s: Upgrading from %d to %d baud...\n", COMP_NAME, from_baud, to_baud);
    fflush(stderr);

    /* Send baud change command to all devices */
    ldcn_cmd_set_baud(&cmd, baud_code);
    ldcn_serial_send_command(old_port, &cmd);
    usleep(500000);  /* 500ms for devices to switch */

    /* Close old port and reopen at new baud */
    ldcn_serial_close(old_port);
    usleep(500000);  /* 500ms settling */

    new_port = ldcn_serial_open(port_name, to_baud);
    if (!new_port) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: Failed to reopen at %d baud\n",
                       COMP_NAME, to_baud);
        return -1;
    }

    usleep(500000);  /* 500ms for serial port to stabilize */
    *port_ptr = new_port;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Reopened at %d baud\n", COMP_NAME, to_baud);
    fprintf(stderr, "%s: Communication upgraded to %d baud\n", COMP_NAME, to_baud);
    fflush(stderr);
    return 0;
}

/* Configure supervisor for full status reporting */
static int configure_supervisor(ldcn_serial_port_t *port) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Configuring supervisor (device %d)...\n",
                   COMP_NAME, SUPERVISOR_ADDR);
    fprintf(stderr, "%s: Configuring supervisor for status reporting...\n", COMP_NAME);
    fflush(stderr);

    /* Send DEFINE_STATUS with 0xFFFF (all status bits) */
    ldcn_cmd_define_status(&cmd, SUPERVISOR_ADDR, 0xFFFF);
    ldcn_serial_send_command(port, &cmd);

    usleep(1000000);  /* 1 second for configuration */

    fprintf(stderr, "%s: Supervisor configured\n", COMP_NAME);
    fflush(stderr);

    return 0;
}

/* Wait for human to press power button and monitor for power on */
static int wait_for_power(ldcn_serial_port_t *port) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status;
    uint8_t last_status = 0x00;
    int power_detected = 0;

    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: ======================================================================\n",
                   COMP_NAME);
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: *** PRESS POWER BUTTON NOW (waiting for power on) ***\n",
                   COMP_NAME);
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: ======================================================================\n",
                   COMP_NAME);

    /* Read baseline status */
    ldcn_build_command(&cmd, SUPERVISOR_ADDR, LDCN_CMD_NOP, NULL, 0);
    for (int attempt = 0; attempt < 5; attempt++) {
        int ret = ldcn_serial_exchange(port, &cmd, &status, 200);
        if (ret > 1 && status.data_len > 0) {
            /* Status byte is data[0] (Python response[1], the 2nd byte) */
            last_status = status.data[0];
            rtapi_print_msg(RTAPI_MSG_INFO, "%s: Baseline status: 0x%02X\n",
                          COMP_NAME, last_status);
            break;
        }
        usleep(200000);
    }

    fprintf(stderr, "%s: Waiting for power button press...\n", COMP_NAME);
    fprintf(stderr, "%s: (Press Ctrl+C to abort)\n", COMP_NAME);
    fflush(stderr);

    /* Poll indefinitely until power on or external abort (Ctrl+C) */
    /* Note: E-stop monitoring is NOT needed here - it's a hardware safety function.
     * The supervisor hardware prevents power-on when E-stop is active. */
    int poll_count = 0;
    while (!done) {  /* done flag set by signal handler */
        ldcn_build_command(&cmd, SUPERVISOR_ADDR, LDCN_CMD_NOP, NULL, 0);
        int ret = ldcn_serial_exchange(port, &cmd, &status, 200);

        if (ret > 1 && status.data_len > 0) {
            /* Status byte is data[0] - this is the diagnostic code from the manual */
            uint8_t current_status = status.data[0];
            uint8_t diagnostic_code = current_status;
            const char *description = ldcn_get_diagnostic_description(diagnostic_code);

            /* Print every 10th poll or when status changes */
            if (poll_count % 10 == 0 || current_status != last_status) {
                rtapi_print_msg(RTAPI_MSG_INFO, "%s: [%3d] Diagnostic: 0x%02X \"%s\"\n",
                              COMP_NAME, poll_count, diagnostic_code, description);
            }

            /* Check if status changed from baseline (power button was pressed)
             * Baseline is typically 0x00 or 0x04 (Power OFF), button changes to 0x08+ */
            if (current_status != last_status && diagnostic_code >= 0x08) {
                fprintf(stderr, "%s: Power button pressed - diagnostic: 0x%02X \"%s\"\n",
                       COMP_NAME, diagnostic_code, description);
                fflush(stderr);
                power_detected = 1;
                break;
            }

            last_status = current_status;
        }

        poll_count++;
        usleep(500000);  /* Poll every 500ms */
    }

    /* Check if we exited due to signal */
    if (done) {
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: ✗ Abort signal received during power wait\n", COMP_NAME);
        return -1;
    }

    if (power_detected) {
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: ✓ Power detected successfully\n", COMP_NAME);
        return 0;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ✗ Power not detected (timeout)\n", COMP_NAME);
        return -1;
    }
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

    /* Step 1: Set address (skip if already addressed during full_init) */
    if (!hal_data->do_full_init) {
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
    }
    
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
    /* Gains from HAL parameters, with defaults matching reference config */
    gains.kp = axis->kp ? axis->kp : 10;     /* Position gain from reference */
    gains.kd = axis->kd ? axis->kd : 1000;   /* Velocity gain from reference */
    gains.ki = axis->ki ? axis->ki : 20;     /* Integral gain from reference */
    gains.il = 40;                            /* Integration limit from reference */
    gains.ol = 255;                           /* Output limit - 100% from reference */
    gains.cl = 0;                             /* Current limit - disabled */
    gains.el = 2000;                          /* Position error limit from reference */
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

    /* Step 6: Reset position counter
     * Sets the position counter to zero */
    ldcn_cmd_reset_position(&cmd, axis->ldcn_addr);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                       "%s: Failed to reset position for axis %d (continuing anyway)\n",
                       COMP_NAME, axis_num);
    }

    /* Step 7: Clear sticky status bits (including position_error flag)
     * Per LS-231SE manual: "In the status byte, the move_done and pos_error
     * flags will be set" after reset. Clear these sticky bits. */
    ldcn_cmd_clear_bits(&cmd, axis->ldcn_addr);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN,
                       "%s: Failed to clear status bits for axis %d (continuing anyway)\n",
                       COMP_NAME, axis_num);
    }

    /* Step 8: Read status to update axis->status with cleared flags */
    usleep(10000);  /* 10ms delay to let drive process the reset */
    ldcn_cmd_read_status(&cmd, axis->ldcn_addr, status_bits);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret > 0) {
        if (ldcn_parse_status(&status, &axis->status, status_bits)) {
            rtapi_print_msg(RTAPI_MSG_INFO,
                           "%s: Axis %d status after reset: pos_err=%d, current_limit=%d, servo_on=%d\n",
                           COMP_NAME, axis_num, axis->status.position_error,
                           axis->status.current_limit, axis->status.servo_on);
        }
    }

    /* Step 9: Send START_MOTION command to activate motion control
     * This is required before the drive will respond to position commands */
    ldcn_cmd_start_motion(&cmd, axis->ldcn_addr);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status, 100);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: Failed to start motion for axis %d\n",
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
    
    /* Handle enable/disable transitions
     * NOTE: Drives are already enabled during initialization with STOP_AMP_ENABLE.
     * We don't need to send enable commands again - just start/stop sending position commands.
     * Only send STOP_MOTOR_OFF when explicitly disabling. */
    if (*axis->enable && !axis->enabled_prev) {
        /* Enable transition - just update state, drive is already enabled from init */
        axis->enabled_prev = true;
    } else if (!*axis->enable && axis->enabled_prev) {
        /* Disable transition - turn motor off */
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

        /* Debug: Log first few position commands for each axis */
        static int cmd_count[32] = {0};
        if (cmd_count[axis_num] < 5) {
            fprintf(stderr, "ldcn: Axis %d CMD: pos=%d counts (%.4f mm), vel=%u, accel=%u\n",
                    axis_num, pos_counts, *axis->pos_cmd, vel_raw, accel_raw);
            fflush(stderr);
            cmd_count[axis_num]++;
        }

        ldcn_cmd_load_trajectory(&cmd, axis->ldcn_addr, &traj);
        ldcn_serial_send_command(hal_data->port, &cmd);
    }
    
    /* Read status */
    uint16_t status_bits = LDCN_STATUS_SEND_POS | LDCN_STATUS_SEND_VEL |
                          LDCN_STATUS_SEND_AUX | LDCN_STATUS_SEND_POS_ERR;
    ldcn_cmd_read_status(&cmd, axis->ldcn_addr, status_bits);

    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status_pkt, 50);
    if (ret > 0) {
        /* CRITICAL: Verify checksum before using data */
        if (!ldcn_verify_checksum(&status_pkt)) {
            static int axis_checksum_errors[32] = {0};
            if (++axis_checksum_errors[axis_num] < 5) {
                /* Calculate expected checksum for debugging */
                uint8_t calc_cksum = status_pkt.status;
                for (int i = 0; i < status_pkt.data_len; i++) {
                    calc_cksum += status_pkt.data[i];
                }
                fprintf(stderr, "%s: !!! CHECKSUM ERROR !!! Axis %d status discarded (count=%d)\n",
                        COMP_NAME, axis_num, axis_checksum_errors[axis_num]);
                fprintf(stderr, "  status=0x%02X, data_len=%d, calc_cksum=0x%02X, recv_cksum=0x%02X, diff=%d\n",
                        status_pkt.status, status_pkt.data_len, calc_cksum, status_pkt.checksum,
                        (int)status_pkt.checksum - (int)calc_cksum);
                /* Dump first few data bytes */
                fprintf(stderr, "  data: ");
                for (int i = 0; i < (status_pkt.data_len < 8 ? status_pkt.data_len : 8); i++) {
                    fprintf(stderr, "%02X ", status_pkt.data[i]);
                }
                fprintf(stderr, "\n");
                fflush(stderr);
            }
            *axis->fault = true;  /* Set fault on checksum error */
            return;
        }

        /* Parse status */
        if (ldcn_parse_status(&status_pkt, &axis->status, status_bits)) {
            /* Update HAL pins with feedback */
            *axis->pos_fb = (hal_float_t)axis->status.position / axis->scale;
            *axis->vel_fb = ldcn_raw_to_velocity(axis->status.velocity,
                                                hal_data->servo_rate_divisor) / axis->scale;
            *axis->ferror = (hal_float_t)axis->status.following_error / axis->scale;
            *axis->amp_enabled = axis->status.servo_on;
            *axis->fault = axis->status.position_error || axis->status.current_limit;

            /* Debug: Log first few status reads for each axis */
            static int status_count[32] = {0};
            if (status_count[axis_num] < 5 || *axis->fault) {
                fprintf(stderr, "ldcn: Axis %d STATUS: pos=%d (%.4f mm), vel=%d, ferror=%d, servo_on=%d, fault=%d (pos_err=%d, cur_lim=%d)\n",
                        axis_num, axis->status.position, *axis->pos_fb,
                        axis->status.velocity, axis->status.following_error,
                        axis->status.servo_on, *axis->fault,
                        axis->status.position_error, axis->status.current_limit);
                fflush(stderr);
                if (!*axis->fault) status_count[axis_num]++;
            }
        }
    }
}

/* Update supervisor status and E-stop monitoring */
static void update_supervisor(void) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status_pkt;
    int ret;

    /* Use NOP command to read supervisor status, same as wait_for_power() */
    ldcn_build_command(&cmd, SUPERVISOR_ADDR, LDCN_CMD_NOP, NULL, 0);
    ret = ldcn_serial_exchange(hal_data->port, &cmd, &status_pkt, 50);

    if (ret > 1 && status_pkt.data_len > 0) {
        /* CRITICAL: Verify checksum before using data */
        if (!ldcn_verify_checksum(&status_pkt)) {
            static int checksum_error_count = 0;
            if (++checksum_error_count < 10) {
                fprintf(stderr, "%s: !!! CHECKSUM ERROR !!! Supervisor status discarded (count=%d)\n",
                        COMP_NAME, checksum_error_count);
                fprintf(stderr, "  status=0x%02X, data[0]=0x%02X, calc_cksum=0x%02X, recv_cksum=0x%02X\n",
                        status_pkt.status, status_pkt.data[0],
                        (uint8_t)(status_pkt.status + status_pkt.data[0]), status_pkt.checksum);
                fflush(stderr);
            }
            return;  /* Discard corrupted data */
        }

        /* Supervisor status byte is in data[0], NOT in status field!
         * The status byte IS the diagnostic code from the manual - use it as-is.
         * LED diagnostic display shows the same code. */
        uint8_t diagnostic_code = status_pkt.data[0];
        const char *description = ldcn_get_diagnostic_description(diagnostic_code);

        /* Debug: Log diagnostic code changes */
        static uint8_t last_code = 0xFF;
        if (diagnostic_code != last_code) {
            fprintf(stderr, "%s: Supervisor diagnostic: 0x%02X \"%s\"\n",
                    COMP_NAME, diagnostic_code, description);
            fflush(stderr);
            last_code = diagnostic_code;
        }

        /* Digital inputs would be in data bytes if we requested them, but with NOP we don't get them
         * Set to 0 since we're not reading them */
        *hal_data->din_raw = 0;

        /* E-stop state logic:
         * BEFORE power-up: Always report estop_ok = TRUE (allow F1/F2 to work)
         * AFTER power-up: Check if motors are powered and system is in good state */
        bool prev_estop_ok = *hal_data->estop_ok;
        if (hal_data->power_up_needed) {
            /* Before power button pressed - allow E-STOP to be cleared */
            *hal_data->estop_ok = true;
        } else {
            /* After power-up - monitor supervisor diagnostic code
             * System is OK if we're in operational states (not faulted):
             * - 0x0C: Cover open, not homed (allows manual jogging)
             * - 0x12-0x14: Ready/Limit/E-Stop states
             * - 0x18-0x1F: Ready and Test mode states
             * - 0x22: POWERED (motors enabled) */
            *hal_data->estop_ok = (diagnostic_code == 0x0C ||  /* Cover open, not homed */
                                   (diagnostic_code >= 0x12 && diagnostic_code <= 0x14) ||  /* Limit/E-Stop */
                                   (diagnostic_code >= 0x18 && diagnostic_code <= 0x22));   /* Ready/Test/Powered */
        }

        /* Debug: Print supervisor status on every change or periodically */
        static int debug_counter = 0;
        static uint8_t last_diagnostic_code = 0xFF;
        if (++debug_counter % 1000 == 0 || diagnostic_code != last_diagnostic_code) {
            if (hal_data->power_up_needed) {
                fprintf(stderr, "%s: Supervisor [%d]: Waiting for F2 press (estop_ok=TRUE, allowing F1/F2)\n",
                        COMP_NAME, debug_counter);
            } else {
                fprintf(stderr, "%s: Supervisor [%d]: 0x%02X \"%s\" (estop_ok=%d)\n",
                        COMP_NAME, debug_counter, diagnostic_code,
                        description, *hal_data->estop_ok);
            }
            fflush(stderr);
            last_diagnostic_code = diagnostic_code;
        }

        /* E-stop monitoring only active AFTER power-up sequence completes */
        if (!hal_data->power_up_needed) {
            /* Print message when supervisor state changes to fault/stop conditions */
            if (prev_estop_ok && !*hal_data->estop_ok) {
                fprintf(stderr, "%s: !!! FAULT DETECTED !!! Supervisor: 0x%02X \"%s\"\n",
                        COMP_NAME, diagnostic_code, description);
                fflush(stderr);
            } else if (!prev_estop_ok && *hal_data->estop_ok) {
                fprintf(stderr, "%s: System OK - Supervisor: 0x%02X \"%s\"\n",
                        COMP_NAME, diagnostic_code, description);
                fflush(stderr);
            }

            /* If supervisor reports fault/stop condition, disable all axes */
            if (!*hal_data->estop_ok) {
                for (int i = 0; i < hal_data->num_axes; i++) {
                    axis_data_t *axis = &hal_data->axes[i];
                    if (axis->initialized && *axis->enable) {
                        /* Send motor off command */
                        ldcn_cmd_stop_motor(&cmd, axis->ldcn_addr, LDCN_STOP_MOTOR_OFF);
                        ldcn_serial_send_command(hal_data->port, &cmd);
                        /* Note: Don't modify enable pin - that's user's input
                         * Just force the motor off in hardware */
                    }
                }
            }
        }
    } else {
        /* Communication failed - fall through to error handling below */
    }

    /* Handle communication errors */
    if (ret <= 0) {
        static int comm_fail_counter = 0;
        if (++comm_fail_counter < 5) {
            fprintf(stderr, "%s: ERROR: Failed to read supervisor status (ret=%d)\n",
                    COMP_NAME, ret);
            fflush(stderr);
        }
    }

    /* Update homing-active pin: TRUE if any axis is homing */
    bool any_homing = false;
    for (int i = 0; i < hal_data->num_axes; i++) {
        axis_data_t *axis = &hal_data->axes[i];
        if (axis->initialized && axis->status.home_in_progress) {
            any_homing = true;
            break;
        }
    }
    *hal_data->homing_active = any_homing;
}

/* Main update loop */
static void update_all(void) {
    ldcn_cmd_packet_t cmd;
    ldcn_status_packet_t status_pkt;

    /* Check if power-up sequence needs to be triggered */
    if (hal_data->power_up_needed && !hal_data->power_up_in_progress) {
        /* Check if ANY axis enable pin is TRUE (F2 pressed) */
        bool enable_requested = false;
        for (int i = 0; i < hal_data->num_axes; i++) {
            if (*hal_data->axes[i].enable) {
                enable_requested = true;
                break;
            }
        }

        if (enable_requested) {
            fprintf(stderr, "\n%s: ======================================================================\n", COMP_NAME);
            fprintf(stderr, "%s: F2 DETECTED - Starting power-up sequence\n", COMP_NAME);
            fprintf(stderr, "%s: *** PRESS POWER BUTTON NOW ***\n", COMP_NAME);
            fprintf(stderr, "%s: ======================================================================\n\n", COMP_NAME);
            fflush(stderr);

            hal_data->power_up_in_progress = true;
            hal_data->waiting_for_power_button = true;

            /* Read baseline status */
            ldcn_build_command(&cmd, SUPERVISOR_ADDR, LDCN_CMD_NOP, NULL, 0);
            int ret = ldcn_serial_exchange(hal_data->port, &cmd, &status_pkt, 200);
            if (ret > 1 && status_pkt.data_len > 0) {
                hal_data->power_button_baseline = status_pkt.data[0];
                fprintf(stderr, "%s: Baseline status: 0x%02X\n", COMP_NAME, hal_data->power_button_baseline);
                fflush(stderr);
            } else {
                hal_data->power_button_baseline = 0x00;
            }
        }
    }

    /* If waiting for power button, poll for it (non-blocking) */
    if (hal_data->waiting_for_power_button) {
        ldcn_build_command(&cmd, SUPERVISOR_ADDR, LDCN_CMD_NOP, NULL, 0);
        int ret = ldcn_serial_exchange(hal_data->port, &cmd, &status_pkt, 200);

        if (ret > 1 && status_pkt.data_len > 0) {
            uint8_t current_status = status_pkt.data[0];
            uint8_t diagnostic_code = current_status;
            const char *description = ldcn_get_diagnostic_description(diagnostic_code);

            /* Check if status changed from baseline (power button was pressed)
             * Baseline is typically 0x00 or 0x04 (Power OFF/Initializing)
             * Button press changes to operational state (0x0C+)
             * We initialize drives as soon as status changes from baseline */
            if (diagnostic_code != hal_data->power_button_baseline && diagnostic_code >= 0x08) {
                fprintf(stderr, "%s: Power button pressed - diagnostic: 0x%02X \"%s\"\n",
                       COMP_NAME, diagnostic_code, description);
                fflush(stderr);

                hal_data->waiting_for_power_button = false;

                /* Initialize all servo drives */
                fprintf(stderr, "%s: Initializing %d servo drive(s)...\n", COMP_NAME, hal_data->num_axes);
                fflush(stderr);
                for (int i = 0; i < hal_data->num_axes; i++) {
                    if (init_drive(i) < 0) {
                        fprintf(stderr, "%s: ERROR: Failed to initialize axis %d\n", COMP_NAME, i);
                        fflush(stderr);
                        hal_data->power_up_in_progress = false;
                        /* Keep power_up_needed = true to allow retry */
                        return;
                    }
                    fprintf(stderr, "%s: Axis %d initialized\n", COMP_NAME, i);
                    fflush(stderr);

                    /* Poll supervisor after each axis to:
                     * 1. Keep communication alive (prevent timeout)
                     * 2. Monitor for current/power issues during init */
                    update_supervisor();

                    /* Small delay between axes to reduce current surge */
                    usleep(50000);  /* 50ms */
                }

                /* Clear any sticky fault bits from initialization on all drives */
                fprintf(stderr, "%s: Clearing fault flags on all drives...\n", COMP_NAME);
                fflush(stderr);
                for (int i = 0; i < hal_data->num_axes; i++) {
                    axis_data_t *axis = &hal_data->axes[i];
                    if (axis->initialized) {
                        ldcn_cmd_packet_t clear_cmd;
                        ldcn_status_packet_t clear_status;
                        ldcn_cmd_clear_bits(&clear_cmd, axis->ldcn_addr);
                        ldcn_serial_exchange(hal_data->port, &clear_cmd, &clear_status, 100);
                    }
                }

                /* Sync position feedback to LinuxCNC - prevents following errors
                 * LinuxCNC's commands may have drifted during the blocking init.
                 * Match feedback to command to absorb the drift. */
                for (int i = 0; i < hal_data->num_axes; i++) {
                    axis_data_t *axis = &hal_data->axes[i];
                    if (axis->initialized) {
                        /* Sync feedback to whatever LinuxCNC is commanding */
                        *axis->pos_fb = *axis->pos_cmd;
                        *axis->vel_fb = 0.0;
                    }
                }

                /* Power-up complete */
                hal_data->power_up_needed = false;
                hal_data->power_up_in_progress = false;

                fprintf(stderr, "%s: All servo drives initialized and ready\n", COMP_NAME);
                fprintf(stderr, "%s: ======================================================================\n\n", COMP_NAME);
                fflush(stderr);
            }
        }

        /* Continue to update supervisor and axes even while waiting */
    }

    /* Update supervisor and E-stop status */
    update_supervisor();

    /* Update all axes
     * Note: update_axis() checks axis->initialized internally,
     * so it's safe to call even during power-up sequence */
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
    axis->kp = 10;    /* Conservative default - tune per axis to avoid oscillation */

    ret = hal_param_u32_newf(HAL_RW, &axis->kd, hal_data->comp_id,
                            "%s.%d.kd", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->kd = 1000;  /* Velocity gain from reference */

    ret = hal_param_u32_newf(HAL_RW, &axis->ki, hal_data->comp_id,
                            "%s.%d.ki", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->ki = 20;    /* Integral gain from reference */

    ret = hal_param_u32_newf(HAL_RO, &axis->address, hal_data->comp_id,
                            "%s.%d.address", COMP_NAME, num);
    if (ret != 0) return ret;
    axis->address = axis->ldcn_addr;
    
    return 0;
}

/* Print usage */
static void usage(void) {
    printf("Usage: ldcn [options]\n");
    printf("\nConfiguration:\n");
    printf("  port=<device>        Serial port (default: %s)\n", DEFAULT_PORT);
    printf("  baud=<rate>          Target baud rate (default: %d)\n", DEFAULT_BAUD);
    printf("  axes=<count>         Number of servo axes (default: %d)\n", DEFAULT_AXES);
    printf("  servo_rate=<divisor> Servo rate divisor (default: %d)\n", DEFAULT_SERVO_RATE);
    printf("  path_mode=<0|1>      Enable path mode (default: 0)\n");
    printf("  path_buffer_freq=<n> Path buffer frequency (default: %d)\n", DEFAULT_PATH_BUFFER_FREQ);
    printf("\nInitialization Phases (test individually or combine):\n");
    printf("  --detect-baud        Auto-detect current baud rate\n");
    printf("  --reset              Hard reset all LDCN devices\n");
    printf("  --discover           Query devices on network\n");
    printf("  --assign             Assign addresses to %d devices\n", TOTAL_LDCN_DEVICES);
    printf("  --upgrade-baud       Upgrade to target baud rate\n");
    printf("  --configure-super    Configure supervisor for full status\n");
    printf("  --wait-power         Wait for power button press (HIIL)\n");
    printf("  --init-servos        Initialize servo drives\n");
    printf("  --full-init          Run complete initialization (default)\n");
    printf("\nExamples:\n");
    printf("  ldcn port=/dev/ttyUSB0           # Full initialization\n");
    printf("  ldcn --detect-baud               # Just detect baud rate\n");
    printf("  ldcn --reset --discover          # Reset and discover devices\n");
    printf("  ldcn --wait-power                # Just wait for power\n");
}

/* Parse command line arguments */
static int parse_args(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "port=", 5) == 0) {
            strncpy(hal_data->port_name, &argv[i][5], sizeof(hal_data->port_name) - 1);
        } else if (strncmp(argv[i], "baud=", 5) == 0) {
            hal_data->baud_rate = atoi(&argv[i][5]);
        } else if (strncmp(argv[i], "debug=", 6) == 0) {
            hal_data->debug_level = atoi(&argv[i][6]);
            if (hal_data->debug_level < 0 || hal_data->debug_level > 2) {
                fprintf(stderr, "Error: debug must be 0 (errors), 1 (normal), or 2 (verbose)\n");
                return -1;
            }
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
        } else if (strcmp(argv[i], "--detect-baud") == 0) {
            hal_data->do_detect_baud = true;
        } else if (strcmp(argv[i], "--reset") == 0) {
            hal_data->do_reset = true;
        } else if (strcmp(argv[i], "--discover") == 0) {
            hal_data->do_discover = true;
        } else if (strcmp(argv[i], "--assign") == 0) {
            hal_data->do_assign = true;
        } else if (strcmp(argv[i], "--upgrade-baud") == 0) {
            hal_data->do_upgrade_baud = true;
        } else if (strcmp(argv[i], "--configure-super") == 0) {
            hal_data->do_configure_supervisor = true;
        } else if (strcmp(argv[i], "--wait-power") == 0) {
            hal_data->do_wait_power = true;
        } else if (strcmp(argv[i], "--init-servos") == 0) {
            hal_data->do_init_servos = true;
        } else if (strcmp(argv[i], "--full-init") == 0) {
            hal_data->do_full_init = true;
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
    int comp_id;

    /* Allocate temporary HAL data structure for argument parsing */
    hal_data = calloc(1, sizeof(hal_data_t));
    if (!hal_data) {
        fprintf(stderr, "%s: ERROR: Failed to allocate memory\n", COMP_NAME);
        return -1;
    }

    /* Set defaults */
    strncpy(hal_data->port_name, DEFAULT_PORT, sizeof(hal_data->port_name) - 1);
    hal_data->baud_rate = DEFAULT_BAUD;
    hal_data->debug_level = 1;  /* Default to normal messages */
    hal_data->num_axes = DEFAULT_AXES;
    hal_data->servo_rate_divisor = DEFAULT_SERVO_RATE;
    hal_data->path_mode = false;
    hal_data->path_buffer_freq = DEFAULT_PATH_BUFFER_FREQ;

    /* Initialize phase control flags */
    hal_data->do_detect_baud = false;
    hal_data->do_reset = false;
    hal_data->do_discover = false;
    hal_data->do_assign = false;
    hal_data->do_upgrade_baud = false;
    hal_data->do_configure_supervisor = false;
    hal_data->do_wait_power = false;
    hal_data->do_init_servos = false;
    hal_data->do_full_init = false;

    /* Parse command line */
    if (parse_args(argc, argv) < 0) {
        free(hal_data);
        return -1;
    }

    /* Set debug level for serial communication */
    ldcn_serial_set_debug(hal_data->debug_level);

    /* Setup signal handlers early so Ctrl+C works during initialization */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Determine initialization mode: if no specific phases requested, do full init */
    if (!hal_data->do_detect_baud && !hal_data->do_reset &&
        !hal_data->do_discover && !hal_data->do_assign &&
        !hal_data->do_upgrade_baud && !hal_data->do_configure_supervisor &&
        !hal_data->do_wait_power && !hal_data->do_init_servos) {
        hal_data->do_full_init = true;
        /* NOTE: Power-wait and servo init are deferred until F2 is pressed.
         * This allows LinuxCNC GUI to start first, then operator presses F2
         * to trigger power button wait sequence. */
    }

    /* Initialize HAL */
    comp_id = hal_init(COMP_NAME);
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", COMP_NAME);
        free(hal_data);
        return -1;
    }

    /* Now allocate HAL data structure in shared memory */
    hal_data_t *temp_data = hal_data;
    hal_data = hal_malloc(sizeof(hal_data_t));
    if (!hal_data) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", COMP_NAME);
        hal_exit(comp_id);
        free(temp_data);
        return -1;
    }

    /* Copy parsed configuration to shared memory */
    memcpy(hal_data, temp_data, sizeof(hal_data_t));
    free(temp_data);
    hal_data->comp_id = comp_id;

    /* Allocate axis data in shared memory */
    hal_data->axes = hal_malloc(hal_data->num_axes * sizeof(axis_data_t));
    if (!hal_data->axes) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for axes\n", COMP_NAME);
        hal_exit(hal_data->comp_id);
        return -1;
    }
    memset(hal_data->axes, 0, hal_data->num_axes * sizeof(axis_data_t));

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
            /* Note: hal_data and axes allocated with hal_malloc, freed by hal_exit */
            return -1;
        }
    }

    /* Export global E-stop monitoring pins */
    ret = hal_pin_bit_newf(HAL_OUT, &hal_data->estop_ok, hal_data->comp_id,
                          "%s.estop-ok", COMP_NAME);
    if (ret != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Failed to export estop-ok pin\n", COMP_NAME);
        hal_exit(hal_data->comp_id);
        return -1;
    }
    *hal_data->estop_ok = 0;  /* Start as FALSE until power is on */

    ret = hal_pin_u32_newf(HAL_OUT, &hal_data->din_raw, hal_data->comp_id,
                          "%s.din-raw", COMP_NAME);
    if (ret != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Failed to export din-raw pin\n", COMP_NAME);
        hal_exit(hal_data->comp_id);
        return -1;
    }
    *hal_data->din_raw = 0;

    ret = hal_pin_bit_newf(HAL_OUT, &hal_data->homing_active, hal_data->comp_id,
                          "%s.homing-active", COMP_NAME);
    if (ret != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Failed to export homing-active pin\n", COMP_NAME);
        hal_exit(hal_data->comp_id);
        return -1;
    }
    *hal_data->homing_active = 0;

    /* Ready */
    hal_ready(hal_data->comp_id);
    
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: Initialized with %d axes on %s at %d baud\n",
                   COMP_NAME, hal_data->num_axes,
                   hal_data->port_name, hal_data->baud_rate);
    
    /*
     * LDCN Modular Initialization:
     * Each phase can be run individually or as part of full initialization.
     * Full sequence: detect baud → reset → discover → assign → upgrade baud →
     *                configure supervisor → wait for power → init servos
     */

    /* Phase 1: Auto-detect current baud rate */
    /* Always detect during full_init for robust reset, or when explicitly requested */
    int detected_baud = 19200;  /* Default to LDCN protocol default */
    if (hal_data->do_detect_baud || hal_data->do_full_init) {
        detected_baud = detect_baud_rate(hal_data->port_name);
        if (detected_baud < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Baud rate detection failed\n", COMP_NAME);
            hal_exit(hal_data->comp_id);
            return -1;
        }
        /* For standalone detect, just report and exit */
        if (hal_data->do_detect_baud && !hal_data->do_full_init) {
            hal_exit(hal_data->comp_id);
            return 0;
        }
    }

    /* Open serial port at detected or default baud */
    rtapi_print_msg(RTAPI_MSG_INFO,
                   "%s: Opening %s at %d baud\n",
                   COMP_NAME, hal_data->port_name, detected_baud);
    hal_data->port = ldcn_serial_open(hal_data->port_name, detected_baud);
    if (!hal_data->port) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "%s: ERROR: Failed to open serial port %s\n",
                       COMP_NAME, hal_data->port_name);
        hal_exit(hal_data->comp_id);
        return -1;
    }

    /* Phase 2: Hard reset network (resets all devices to 19200 baud) */
    /* For robust reset: if devices not at 19200, reset at current baud first */
    if (hal_data->do_reset || hal_data->do_full_init) {
        /* If devices are at a different baud, send reset at that baud first */
        if (detected_baud != 19200 && detected_baud > 0) {
            rtapi_print_msg(RTAPI_MSG_INFO,
                           "%s: Sending hard reset at detected baud %d...\n",
                           COMP_NAME, detected_baud);
            ret = hard_reset_network(hal_data->port);
            if (ret < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Network reset failed\n", COMP_NAME);
                ldcn_serial_close(hal_data->port);
                hal_exit(hal_data->comp_id);
                return -1;
            }

            /* Close and reopen at 19200 for second reset */
            ldcn_serial_close(hal_data->port);
            usleep(1000000);  /* 1s delay */

            rtapi_print_msg(RTAPI_MSG_INFO,
                           "%s: Reopening at 19200 baud...\n", COMP_NAME);
            hal_data->port = ldcn_serial_open(hal_data->port_name, 19200);
            if (!hal_data->port) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                               "%s: ERROR: Failed to reopen serial port\n",
                               COMP_NAME);
                hal_exit(hal_data->comp_id);
                return -1;
            }
        }

        /* Now send hard reset at 19200 to ensure clean state */
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Sending hard reset at 19200 baud...\n", COMP_NAME);
        ret = hard_reset_network(hal_data->port);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Network reset failed\n", COMP_NAME);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            return -1;
        }
    }

    /* Phase 3: Discover devices on network (skip after reset - devices at address 0) */
    int num_devices = TOTAL_LDCN_DEVICES;  /* Default assumption */
    if (hal_data->do_discover && !hal_data->do_full_init) {
        /* Discovery only works if devices already have addresses */
        num_devices = discover_devices(hal_data->port, TOTAL_LDCN_DEVICES);
        if (num_devices < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Device discovery failed\n", COMP_NAME);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            return -1;
        }
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Discovered %d LDCN devices\n", COMP_NAME, num_devices);
    }

    /* Phase 4: Assign addresses to all devices */
    if (hal_data->do_assign || hal_data->do_full_init) {
        ret = assign_device_addresses(hal_data->port, num_devices);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Address assignment failed\n", COMP_NAME);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            return -1;
        }
    }

    /* Phase 5: Upgrade baud rate if different from 19200 */
    if ((hal_data->do_upgrade_baud || hal_data->do_full_init) && hal_data->baud_rate != 19200) {
        ret = upgrade_baud_rate(&hal_data->port, hal_data->port_name, 19200, hal_data->baud_rate);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Baud rate upgrade failed\n", COMP_NAME);
            if (hal_data->port) {
                ldcn_serial_close(hal_data->port);
            }
            hal_exit(hal_data->comp_id);
            return -1;
        }
    }

    /* Phase 6: Configure supervisor for full status reporting */
    if (hal_data->do_configure_supervisor || hal_data->do_full_init) {
        ret = configure_supervisor(hal_data->port);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Supervisor configuration failed\n", COMP_NAME);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            return -1;
        }
    }

    /* Phase 7: Wait for power button press (HIIL - Human In The Loop) */
    /* Hardware requires power button press for proper initialization */
    if (hal_data->do_wait_power) {
        ret = wait_for_power(hal_data->port);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Power wait failed or timed out\n", COMP_NAME);
            ldcn_serial_close(hal_data->port);
            hal_exit(hal_data->comp_id);
            return -1;
        }
    }

    /* Phase 8: Initialize servo drives
     * Only do this during startup if explicitly requested (do_init_servos).
     * For full_init mode, servo initialization is deferred until F2 is pressed. */
    if (hal_data->do_init_servos) {
        rtapi_print_msg(RTAPI_MSG_INFO, "%s: Initializing %d servo drives...\n",
                       COMP_NAME, hal_data->num_axes);
        fprintf(stderr, "%s: Initializing %d servo drive(s)...\n", COMP_NAME, hal_data->num_axes);
        fflush(stderr);
        for (int i = 0; i < hal_data->num_axes; i++) {
            if (init_drive(i) < 0) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                              "%s: ERROR: Failed to initialize axis %d\n",
                              COMP_NAME, i);
                ldcn_serial_close(hal_data->port);
                hal_exit(hal_data->comp_id);
                return -1;
            }
            fprintf(stderr, "%s: Axis %d initialized\n", COMP_NAME, i);
            fflush(stderr);
        }
        fprintf(stderr, "%s: All servo drives initialized\n", COMP_NAME);
        fflush(stderr);
    }
    
    /* Setup signal handlers */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Determine if power-up sequence is needed */
    if (hal_data->do_wait_power || hal_data->do_init_servos) {
        /* Explicit power-wait or servo-init requested - already done above */
        hal_data->power_up_needed = false;
    } else if (hal_data->do_full_init) {
        /* Full init mode - defer power-up until F2 pressed */
        hal_data->power_up_needed = true;
        fprintf(stderr, "%s: *** Power-up deferred until F2 is pressed ***\n", COMP_NAME);
        fflush(stderr);
    } else {
        /* Partial init mode - no power-up needed */
        hal_data->power_up_needed = false;
    }
    hal_data->power_up_in_progress = false;
    hal_data->waiting_for_power_button = false;
    hal_data->power_button_baseline = 0x00;

    /* Main loop - run at ~1ms (1000 Hz) */
    hal_data->running = true;
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Entering main loop\n", COMP_NAME);
    fprintf(stderr, "%s: Initialization complete - entering control loop\n", COMP_NAME);
    fprintf(stderr, "%s: Monitoring E-stop and updating axes at 1kHz\n", COMP_NAME);
    fflush(stderr);

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
    /* Note: hal_data and axes allocated with hal_malloc, freed by hal_exit */

    return 0;
}
