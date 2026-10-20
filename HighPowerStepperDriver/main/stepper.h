#pragma once

#include "driver/spi_master.h"
#include <stdint.h>
#include <stdbool.h>

static spi_device_handle_t spi_handle;

// Fixed: Using standard C enum syntax for cross-language safety
typedef enum {
    HPSD_REG_CTRL   = 0x00,
    HPSD_REG_TORQUE = 0x01,
    HPSD_REG_OFF    = 0x02,
    HPSD_REG_BLANK  = 0x03,
    HPSD_REG_DECAY  = 0x04,
    HPSD_REG_STALL  = 0x05,
    HPSD_REG_DRIVE  = 0x06,
    HPSD_REG_STATUS = 0x07
} drv8711_reg_addr_t;

typedef enum {
    MICROSTEP_1   = 0b0000, // Full-step (71% current)
    MICROSTEP_2   = 0b0001, // 1/2 step
    MICROSTEP_4   = 0b0010, // 1/4 step
    MICROSTEP_8   = 0b0011, // 1/8 step
    MICROSTEP_16  = 0b0100, // 1/16 step
    MICROSTEP_32  = 0b0101, // 1/32 step
    MICROSTEP_64  = 0b0110, // 1/64 step
    MICROSTEP_128 = 0b0111, // 1/128 step
    MICROSTEP_256 = 0b1000  // 1/256 step
} drv8711_step_mode_t;

// Function declarations (Prototypes only - No bodies!)
void drv8711_write_register(uint8_t address, uint16_t data);
uint16_t drv8711_read_register(uint8_t address);
void drv8711_set_current_mA(uint16_t current_ma);
void drv8711_set_step_mode(drv8711_step_mode_t mode);
void drv8711_set_decay_mode_auto_mixed(void);
void drv8711_apply_default_profile(void);
