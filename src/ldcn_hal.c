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
 * - Communication status monitoring
 * 
 * Usage:
 *   loadrt ldcn_hal port=/dev/ttyUSB0 num_servos=5
 * 
 * HAL Pins:
 *   ldcn.power-on           (bit, out) - TRUE when machine power is on
 *   ldcn.comms-ok           (bit, out) - TRUE when LDCN communication is working
 *   ldcn.enable-request     (bit, in)  - TRUE to enable servo amplifiers
 *   ldcn.servo.N.enabled    (bit, out) - TRUE when servo N amplifier is enabled
 * 
 * HAL Parameters:
 *   ldcn.baud-rate          (u32, ro)  - Current baud rate
 *   ldcn.comm-errors        (u32, ro)  - Communication error count
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
    
    /* Control inputs */
    hal_bit_t *enable_request;  /* Request to enable drives */
    
    /* Per-servo status */
    struct {
        hal_bit_t *enabled;     /* Amplifier is enabled */
    } servo[MAX_SERVOS];
    
    /* Parameters */
    hal_u32_t *baud_rate;       /* Current baud rate */
    hal_u32_t *comm_errors;     /* Communication error count */
    
} ldcn_hal_t;

/* HAL data structure */
static ldcn_hal_t *hal_data;

/* LDCN network structure */
static ldcn_network_t *ldcn_net = NULL;

/* State tracking */
static bool initialized = false;
static bool last_enable_request = false;
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
 * update_servo_enables - Enable or disable servo amplifiers
 * 
 * This function enables/disables all servo amplifiers based on the
 * enable_request input pin.
 */
static void update_servo_enables(void)
{
    bool enable = *hal_data->enable_request;
    
    if (!ldcn_net || !*hal_data->power_on) {
        /* Can't enable without power */
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
        
        last_enable_request = enable;
    }
}

/**
 * ldcn_read - HAL read function (called periodically by LinuxCNC)
 * 
 * This function:
 * 1. Reads power status from I/O controller
 * 2. Updates communication status
 * 3. Handles servo enable/disable requests
 */
static void ldcn_read(void *arg, long period)
{
    /* Initialize network on first call */
    if (!initialized) {
        if (initialize_network() == 0) {
            initialized = true;
        } else {
            *hal_data->comms_ok = false;
            return;
        }
    }
    
    /* Read power status */
    read_power_status();
    
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
    
    /* Export control pins */
    ret = hal_pin_bit_new("ldcn.enable-request", HAL_IN, &(hal_data->enable_request), comp_id);
    if (ret < 0) goto error;
    
    /* Export per-servo pins */
    for (int i = 0; i < num_servos; i++) {
        char name[HAL_NAME_LEN + 1];
        
        snprintf(name, sizeof(name), "ldcn.servo.%d.enabled", i);
        ret = hal_pin_bit_new(name, HAL_OUT, &(hal_data->servo[i].enabled), comp_id);
        if (ret < 0) goto error;
    }
    
    /* Export parameters */
    ret = hal_param_u32_new("ldcn.baud-rate", HAL_RO, &(hal_data->baud_rate), comp_id);
    if (ret < 0) goto error;
    
    ret = hal_param_u32_new("ldcn.comm-errors", HAL_RO, &(hal_data->comm_errors), comp_id);
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
    *hal_data->enable_request = false;
    *hal_data->baud_rate = 0;
    *hal_data->comm_errors = 0;
    
    for (int i = 0; i < num_servos; i++) {
        *hal_data->servo[i].enabled = false;
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
