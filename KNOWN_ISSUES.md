# Known Issues

## HAL Parameter Export Error (CRITICAL)

**Status:** Blocking basic functionality
**Error:** `HAL: ERROR: data_ptr_addr not in shared memory`

### Description

When attempting to load the ldcn component via halrun, it fails during HAL parameter export:

```
ldcn: ERROR: Failed to export axis 0
```

### Root Cause

The HAL subsystem is rejecting the memory addresses used for parameters (scale, kp, kd, ki, address) because they are not in HAL shared memory. For userspace HAL components, the memory allocation strategy for parameters needs investigation.

### Current Implementation

```c
/* Parameters stored as direct struct members */
typedef struct {
    hal_float_t scale;
    hal_u32_t kp;
    hal_u32_t kd;
    hal_u32_t ki;
    hal_u32_t address;
} axis_data_t;

/* Allocated with calloc() */
hal_data->axes = calloc(hal_data->num_axes, sizeof(axis_data_t));

/* Exported to HAL */
hal_param_float_newf(HAL_RW, &axis->scale, hal_data->comp_id, ...);
```

### Attempted Fixes

1. **hal_malloc()** - Tried allocating axis array with `hal_malloc()` instead of `calloc()` - same error
2. **rtapi_app.h** - Removed incorrect header meant for kernel components - no change

### Next Steps

1. Review other userspace HAL components to understand correct parameter allocation
2. Check if userspace components should use a different HAL init method
3. Verify ULAPI define and userspace-specific HAL functions
4. Consider if parameters should use hal_pin_*_newf instead of hal_param_*_newf
5. Consult LinuxCNC HAL documentation for userspace component requirements

### Impact

**Blocks all testing** - Component cannot be loaded into HAL until this is resolved.

The initialization sequence code (19200 → reset → init → upgrade to 125000) is implemented correctly but cannot be tested.

### Workaround

None currently - this is a fundamental issue with HAL integration.

---

*Last Updated: 2025-10-27*
