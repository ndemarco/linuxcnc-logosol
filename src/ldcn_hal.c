/*
 * ldcn_hal.c - LinuxCNC HAL component for Logosol LDCN network
 *
 * This HAL component provides an interface between LinuxCNC and a Logosol
 * Distributed Control Network (LDCN) consisting of LS-231SE servo drives
 * and a CNC-SK-2310g2 I/O controller.
 *
 * Features:
 * - Automatic network initialization and baud rate switching
 * - Power-on detection from 2310g2 I/O controller
 * - Drive enable/disable control
 * - Emergency stop handling
 * - Fault detection and reporting
 * - Watchdog timer for communication health
 * - Communication status monitoring
 *
 * Usage:
 *   loadrt ldcn port=/dev/ttyUSB0 num_servos=5
 *
 * HAL Pins:
 *   ldcn.power-on           (bit, out) - TRUE when machine power is on
 *   ldcn.comms-ok           (bit, out) - TRUE when LDCN communication is working
 *   ldcn.fault              (bit, out) - TRUE when any servo has a fault
 *   ldcn.enable-request     (bit, in)  - TRUE to enable servo amplifiers
 *   ldcn.estop              (bit, in)  - Emergency stop input (TRUE = stop)
 *   ldcn.servo.N.enabled    (bit, out) - TRUE when servo N amplifier is enabled
 *   ldcn.servo.N.fault      (bit, out) - TRUE when servo N has a fault
 *
 * HAL Parameters:
 *   ldcn.baud-rate          (u32, ro)  - Current baud rate
 *   ldcn.comm-errors        (u32, ro)  - Communication error count
 *   ldcn.watchdog-timeout   (float, rw) - Watchdog timeout in seconds (default: 1.0)
 *
 * Safety Features:
 * - Emergency stop immediately disables all servos
 * - Automatic disable on power loss or faults
 * - Watchdog timer auto-disables on communication timeout
 * - Fault detection for checksum errors, current limit, position errors
 *
 * Copyright (C) 2025
 * License: GPL v2 or later
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "ldcn_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Module information */
MODULE_AUTHOR("LinuxCNC Community");
MODULE_DESCRIPTION("Logosol LDCN Network HAL Component");
MODULE_LICENSE("GPL");

/* Module parameters */
static char *port = "/dev/ttyUSB0";
RTAPI_MP_STRING(port, "Serial port device (default: /dev/ttyUSB0)");

static int num_servos = 5;
RTAPI_MP_INT(num_servos, "Number of servo drives (default: 5, max: 5)");

static int target_baud = 125000;
RTAPI_MP_INT(target_baud, "Target baud rate (default: 125000)");

/* HAL component ID */
static int comp_id;

/* Maximum servo drives (device 6 is I/O controller) */
#define MAX_SERVOS 5

/**
 * HAL shared memory structure
 */
typedef struct {
    /* Status outputs */
    hal_bit_t *power_on;        /* Machine power is on */
    hal_bit_t *comms_ok;        /* Communication is working */
    hal_bit_t *fault;           /* Fault detected on any servo */

    /* Control inputs */
    hal_bit_t *enable_request;  /* Request to enable drives */
    hal_bit_t *estop;           /* Emergency stop input */

    /* Per-servo status */
    struct {
        hal_bit_t *enabled;     /* Amplifier is enabled */
        hal_bit_t *fault;       /* Fault detected on this servo */
    } servo[MAX_SERVOS];

    /* Parameters */
    hal_u32_t *baud_rate;       /* Current baud rate */
    hal_u32_t *comm_errors;     /* Communication error count */
    hal_float_t *watchdog_timeout; /* Watchdog timeout in seconds */

} ldcn_hal_t;

/* HAL data structure */
static ldcn_hal_t *hal_data;

/* LDCN network structure */
static ldcn_network_t *ldcn_net = NULL;

/* State tracking */
static bool initialized = false;
static bool last_enable_request = false;
static bool last_estop = false;
static bool servos_enabled = false;
static long long last_successful_read = 0;  /* Nanoseconds since component start */
static int io_controller_addr = 6;

/**
 * initialize_network - Initialize LDCN network
 * 
 * This function:
 * 1. Auto-detects current baud rate
 * 2. Resets network if needed
 * 3. Addresses all devices
 * 4. Switches to target baud rate
 * 5. Configures I/O controller for full status
 * 
 * Returns: 0 on success, -1 on failure
 */
static int initialize_network(void)
{
    int ret;
    
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Initializing network on %s\n", port);
    
    /* Auto-detect current baud rate */
    int detected_baud = ldcn_auto_detect_baud(port);
    if (detected_baud > 0) {
        rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Detected devices at %d baud\n", detected_baud);
        
        /* If not at 19200, reset to default */
        if (detected_baud != 19200) {
            ldcn_network_t *temp_net = ldcn_open(port, detected_baud);
            if (temp_net) {
                rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Resetting to 19200 baud\n");
                ldcn_hard_reset(temp_net);
                ldcn_close(temp_net);
                rtapi_sleep(1000000000); /* 1 second */
            }
        }
    }
    
    /* Open at 19200 baud */
    ldcn_net = ldcn_open(port, 19200);
    if (!ldcn_net) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: Failed to open %s\n", port);
        return -1;
    }
    
    /* Hard reset */
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Sending hard reset\n");
    ret = ldcn_hard_reset(ldcn_net);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: Hard reset failed\n");
        ldcn_close(ldcn_net);
        ldcn_net = NULL;
        return -1;
    }
    
    /* Address devices (5 servos + 1 I/O controller) */
    int total_devices = num_servos + 1;
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Addressing %d devices\n", total_devices);
    
    ret = ldcn_address_devices(ldcn_net, total_devices);
    if (ret != total_devices) {
        rtapi_print_msg(RTAPI_MSG_WARN, "ldcn_hal: Only %d of %d devices addressed\n",
                       ret, total_devices);
    }
    
    /* Verify at 19200 */
    ret = ldcn_verify_devices(ldcn_net);
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: %d devices responding at 19200 baud\n", ret);
    
    if (ret < total_devices) {
        rtapi_print_msg(RTAPI_MSG_WARN, "ldcn_hal: Not all devices responding\n");
    }
    
    /* Switch to target baud rate */
    if (target_baud != 19200) {
        rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Switching to %d baud\n", target_baud);
        
        ret = ldcn_set_baud_rate(ldcn_net, target_baud);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: Failed to change baud rate\n");
            ldcn_close(ldcn_net);
            ldcn_net = NULL;
            return -1;
        }
        
        /* Verify at new baud */
        ret = ldcn_verify_devices(ldcn_net);
        rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: %d devices responding at %d baud\n",
                       ret, target_baud);
        
        if (ret < total_devices) {
            rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: Communication failed at %d baud\n",
                           target_baud);
            ldcn_close(ldcn_net);
            ldcn_net = NULL;
            return -1;
        }
    }
    
    /* Configure I/O controller (device 6) for full status reporting */
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Configuring I/O controller\n");
    ret = ldcn_define_status(ldcn_net, io_controller_addr, 0xFFFF);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_WARN, "ldcn_hal: Failed to configure I/O controller status\n");
    }
    
    rtapi_sleep(1000000000); /* 1 second settle time */
    
    /* Update HAL parameters */
    *hal_data->baud_rate = ldcn_net->baud_rate;
    *hal_data->comm_errors = ldcn_net->comm_errors;
    
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Network initialized successfully\n");
    
    return 0;
}

/**
 * read_power_status - Read power-on status from I/O controller
 * 
 * Returns: 0 on success, -1 on failure
 */
static int read_power_status(void)
{
    uint8_t status;
    int ret;
    
    if (!ldcn_net) {
        return -1;
    }
    
    ret = ldcn_read_status(ldcn_net, io_controller_addr, &status);
    
    if (ret == LDCN_OK) {
        /* Bit 3 indicates power-on state */
        *hal_data->power_on = (status & LDCN_STATUS_POWER_ON) != 0;
        *hal_data->comms_ok = true;
        return 0;
    } else {
        *hal_data->comms_ok = false;
        *hal_data->comm_errors = ldcn_net->comm_errors;
        return -1;
    }
}

/**
 * check_servo_faults - Check for fault conditions on all servos
 *
 * Returns: Number of servos with faults
 */
static int check_servo_faults(void)
{
    int total_faults = 0;
    bool any_fault = false;

    if (!ldcn_net) {
        return 0;
    }

    for (int i = 0; i < num_servos; i++) {
        int addr = i + 1;  /* Servos are addresses 1-5 */
        uint8_t faults;

        int ret = ldcn_check_faults(ldcn_net, addr, &faults);

        if (ret > 0) {
            /* Faults detected */
            *hal_data->servo[i].fault = true;
            any_fault = true;
            total_faults += ret;

            rtapi_print_msg(RTAPI_MSG_WARN,
                           "ldcn_hal: Servo %d faults detected: 0x%02X\n",
                           i, faults);
        } else if (ret == 0) {
            /* No faults */
            *hal_data->servo[i].fault = false;
        } else {
            /* Communication error - treat as fault */
            *hal_data->servo[i].fault = true;
            any_fault = true;
        }
    }

    *hal_data->fault = any_fault;
    return total_faults;
}

/**
 * handle_emergency_stop - Handle emergency stop condition
 */
static void handle_emergency_stop(void)
{
    if (!ldcn_net) {
        return;
    }

    rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: EMERGENCY STOP activated\n");

    /* Send emergency stop command */
    int ret = ldcn_emergency_stop(ldcn_net);

    if (ret != LDCN_OK) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "ldcn_hal: Emergency stop command failed\n");
    }

    /* Update all servo status */
    for (int i = 0; i < num_servos; i++) {
        *hal_data->servo[i].enabled = false;
    }

    servos_enabled = false;
    last_enable_request = false;
}

/**
 * update_servo_enables - Enable or disable servo amplifiers
 *
 * This function enables/disables all servo amplifiers based on the
 * enable_request input pin.
 */
static void update_servo_enables(void)
{
    bool enable = *hal_data->enable_request;

    if (!ldcn_net || !*hal_data->power_on || *hal_data->fault) {
        /* Can't enable without power or with faults */
        enable = false;
    }

    /* Only update if request changed */
    if (enable != last_enable_request) {
        rtapi_print_msg(RTAPI_MSG_DBG, "ldcn_hal: %s servo amplifiers\n",
                       enable ? "Enabling" : "Disabling");

        /* Update all servo drives */
        for (int i = 0; i < num_servos; i++) {
            int addr = i + 1;  /* Servos are addresses 1-5 */
            int ret = ldcn_enable_amplifier(ldcn_net, addr, enable);

            if (ret == LDCN_OK) {
                *hal_data->servo[i].enabled = enable;
            } else {
                rtapi_print_msg(RTAPI_MSG_ERR,
                               "ldcn_hal: Failed to %s servo %d\n",
                               enable ? "enable" : "disable", i);
                *hal_data->servo[i].enabled = false;
            }
        }

        servos_enabled = enable;
        last_enable_request = enable;
    }
}

/**
 * ldcn_read - HAL read function (called periodically by LinuxCNC)
 *
 * This function:
 * 1. Reads power status from I/O controller
 * 2. Updates communication status
 * 3. Checks for emergency stop
 * 4. Checks for faults
 * 5. Implements watchdog timer
 * 6. Handles servo enable/disable requests
 */
static void ldcn_read(void *arg, long period)
{
    static long long total_time = 0;
    total_time += period;  /* Accumulate time in nanoseconds */

    /* Initialize network on first call */
    if (!initialized) {
        if (initialize_network() == 0) {
            initialized = true;
            last_successful_read = total_time;
        } else {
            *hal_data->comms_ok = false;
            return;
        }
    }

    /* Check for emergency stop */
    if (*hal_data->estop && !last_estop) {
        /* E-stop asserted (rising edge) */
        handle_emergency_stop();
        last_estop = true;
        return;
    }
    last_estop = *hal_data->estop;

    /* If E-stop is active, disable servos and return */
    if (*hal_data->estop) {
        *hal_data->comms_ok = false;
        for (int i = 0; i < num_servos; i++) {
            *hal_data->servo[i].enabled = false;
        }
        return;
    }

    /* Read power status */
    int power_ret = read_power_status();

    /* Update watchdog timer */
    if (power_ret == 0) {
        /* Successful communication */
        last_successful_read = total_time;
    } else {
        /* Communication failed - check watchdog */
        long long timeout_ns = (long long)(*hal_data->watchdog_timeout * 1000000000.0);

        if (timeout_ns > 0 && (total_time - last_successful_read) > timeout_ns) {
            /* Watchdog timeout - disable servos */
            if (servos_enabled) {
                rtapi_print_msg(RTAPI_MSG_ERR,
                               "ldcn_hal: Watchdog timeout - disabling servos\n");
                handle_emergency_stop();
            }
            *hal_data->comms_ok = false;
            return;
        }
    }

    /* Check for faults on all servos */
    check_servo_faults();

    /* Auto-disable on power loss or faults */
    if (!*hal_data->power_on || *hal_data->fault) {
        if (servos_enabled) {
            rtapi_print_msg(RTAPI_MSG_WARN,
                           "ldcn_hal: Auto-disabling servos (power=%d, fault=%d)\n",
                           *hal_data->power_on, *hal_data->fault);
            handle_emergency_stop();
        }
    }

    /* Update servo enables */
    update_servo_enables();
}

/**
 * rtapi_app_main - Module initialization
 */
int rtapi_app_main(void)
{
    int ret;
    
    /* Validate parameters */
    if (num_servos < 1 || num_servos > MAX_SERVOS) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                       "ldcn_hal: num_servos must be 1-%d, got %d\n",
                       MAX_SERVOS, num_servos);
        return -1;
    }
    
    /* Create HAL component */
    comp_id = hal_init("ldcn");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: hal_init failed\n");
        return -1;
    }
    
    /* Allocate HAL shared memory */
    hal_data = hal_malloc(sizeof(ldcn_hal_t));
    if (!hal_data) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: hal_malloc failed\n");
        hal_exit(comp_id);
        return -1;
    }
    
    memset(hal_data, 0, sizeof(ldcn_hal_t));
    
    /* Export status pins */
    ret = hal_pin_bit_new("ldcn.power-on", HAL_OUT, &(hal_data->power_on), comp_id);
    if (ret < 0) goto error;

    ret = hal_pin_bit_new("ldcn.comms-ok", HAL_OUT, &(hal_data->comms_ok), comp_id);
    if (ret < 0) goto error;

    ret = hal_pin_bit_new("ldcn.fault", HAL_OUT, &(hal_data->fault), comp_id);
    if (ret < 0) goto error;

    /* Export control pins */
    ret = hal_pin_bit_new("ldcn.enable-request", HAL_IN, &(hal_data->enable_request), comp_id);
    if (ret < 0) goto error;

    ret = hal_pin_bit_new("ldcn.estop", HAL_IN, &(hal_data->estop), comp_id);
    if (ret < 0) goto error;

    /* Export per-servo pins */
    for (int i = 0; i < num_servos; i++) {
        char name[HAL_NAME_LEN + 1];

        snprintf(name, sizeof(name), "ldcn.servo.%d.enabled", i);
        ret = hal_pin_bit_new(name, HAL_OUT, &(hal_data->servo[i].enabled), comp_id);
        if (ret < 0) goto error;

        snprintf(name, sizeof(name), "ldcn.servo.%d.fault", i);
        ret = hal_pin_bit_new(name, HAL_OUT, &(hal_data->servo[i].fault), comp_id);
        if (ret < 0) goto error;
    }

    /* Export parameters */
    ret = hal_param_u32_new("ldcn.baud-rate", HAL_RO, &(hal_data->baud_rate), comp_id);
    if (ret < 0) goto error;

    ret = hal_param_u32_new("ldcn.comm-errors", HAL_RO, &(hal_data->comm_errors), comp_id);
    if (ret < 0) goto error;

    ret = hal_param_float_new("ldcn.watchdog-timeout", HAL_RW, &(hal_data->watchdog_timeout), comp_id);
    if (ret < 0) goto error;
    
    /* Export read function */
    ret = hal_export_funct("ldcn.read", ldcn_read, hal_data, 1, 0, comp_id);
    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: hal_export_funct failed\n");
        goto error;
    }
    
    /* Initialize default values */
    *hal_data->power_on = false;
    *hal_data->comms_ok = false;
    *hal_data->fault = false;
    *hal_data->enable_request = false;
    *hal_data->estop = false;
    *hal_data->baud_rate = 0;
    *hal_data->comm_errors = 0;
    *hal_data->watchdog_timeout = 1.0;  /* Default 1 second timeout */

    for (int i = 0; i < num_servos; i++) {
        *hal_data->servo[i].enabled = false;
        *hal_data->servo[i].fault = false;
    }
    
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Component loaded\n");
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Port: %s, Servos: %d, Target baud: %d\n",
                   port, num_servos, target_baud);
    
    hal_ready(comp_id);
    return 0;
    
error:
    rtapi_print_msg(RTAPI_MSG_ERR, "ldcn_hal: Failed to export HAL pins/parameters\n");
    hal_exit(comp_id);
    return -1;
}

/**
 * rtapi_app_exit - Module cleanup
 */
void rtapi_app_exit(void)
{
    if (ldcn_net) {
        /* Disable all servos before shutting down */
        for (int i = 0; i < num_servos; i++) {
            ldcn_enable_amplifier(ldcn_net, i + 1, false);
        }
        
        ldcn_close(ldcn_net);
        ldcn_net = NULL;
    }
    
    hal_exit(comp_id);
    rtapi_print_msg(RTAPI_MSG_INFO, "ldcn_hal: Component unloaded\n");
}
