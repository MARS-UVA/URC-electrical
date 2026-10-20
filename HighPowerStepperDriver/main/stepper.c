#include "stepper.h"
#include "esp_log.h"


void drv8711_write_register(uint8_t address, uint16_t data) {
    uint16_t packet = ((uint16_t)(address & 0x07) << 12) | (data & 0x0FFF);
    packet = (packet >> 8) | (packet << 8); // Swap to Big-Endian

    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .tx_buffer = &packet,
        .rx_buffer = NULL
    };
    spi_device_transmit(spi_handle, &t);
}

uint16_t drv8711_read_register(uint8_t address) {
    uint16_t packet = (1 << 15) | ((uint16_t)(address & 0x07) << 12);
    packet = (packet >> 8) | (packet << 8);

    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .tx_buffer = &packet,
        .rx_buffer = &packet // Write-back pattern safely captures read stream
    };
    spi_device_transmit(spi_handle, &t);
    
    packet = (packet >> 8) | (packet << 8); // Restore Little-Endian
    return packet & 0x0FFF;
}

void drv8711_set_current_mA(uint16_t current_ma) {
    if (current_ma > 16000) { 
        current_ma = 16000; 
    }
    
    uint8_t isgain_bits = 0b11; 
    uint32_t torque_bits = (384 * current_ma) / 6875;

    while (torque_bits > 0xFF) {
        isgain_bits--;
        torque_bits >>= 1;
    }

    uint16_t current_ctrl = drv8711_read_register(HPSD_REG_CTRL);
    current_ctrl = (current_ctrl & 0b110011111111) | (isgain_bits << 8);
    drv8711_write_register(HPSD_REG_CTRL, current_ctrl);

    uint16_t current_torque = drv8711_read_register(HPSD_REG_TORQUE);
    current_torque = (current_torque & 0b111100000000) | (torque_bits & 0xFF);
    drv8711_write_register(HPSD_REG_TORQUE, current_torque);
}

void drv8711_set_step_mode(drv8711_step_mode_t mode) {
    uint16_t current_ctrl = drv8711_read_register(HPSD_REG_CTRL);
    current_ctrl = (current_ctrl & 0b111110000111) | (mode << 3);
    drv8711_write_register(HPSD_REG_CTRL, current_ctrl);
}

void drv8711_set_decay_mode_auto_mixed(void) {
    uint16_t current_decay = drv8711_read_register(HPSD_REG_DECAY);
    current_decay = (current_decay & 0b00011111111) | (0b101 << 8); 
    drv8711_write_register(HPSD_REG_DECAY, current_decay);
}

void drv8711_apply_default_profile(void) {
    drv8711_write_register(HPSD_REG_OFF, 0x030);   
    drv8711_write_register(HPSD_REG_BLANK, 0x080); 
    drv8711_write_register(HPSD_REG_STALL, 0x040); 
    drv8711_write_register(HPSD_REG_DRIVE, 0xA59); 
    
    // Fixed: calling corrected syntax targets directly
    drv8711_set_current_mA(2000);  
    drv8711_set_step_mode(MICROSTEP_1);   
    drv8711_set_decay_mode_auto_mixed();   
    
    uint16_t current_ctrl = drv8711_read_register(HPSD_REG_CTRL);
    drv8711_write_register(HPSD_REG_CTRL, current_ctrl | (1 << 0)); 
}
