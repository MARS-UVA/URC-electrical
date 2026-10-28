#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "stepper.h"

// SPI pins
#define PIN_NUM_MISO    17
#define PIN_NUM_MOSI    16
#define PIN_NUM_CLK     10
#define CS1             9

#define PIN_NUM_STEP    12
#define PIN_NUM_DIR     11
#define PIN_NUM_SLEEP   8
#define PIN_NUM_FAULT   15
#define PIN_NUM_RST     5

#define PIN_NUM_INPUT   4 // button pin
#define DEBOUNCE_DELAY_US 200000ULL // debounce delay in ms (200ms)

// this period is length of the delay between steps
// increase delay -- slower
// decrease delay -- faster, but there is a limit before 
// it starts skipping steps
#define STEP_PERIOD_US 1000

static const char *TAG = "STEPPER";

static QueueHandle_t button_queue;
static volatile uint64_t last_isr_time = 0;
static volatile uint32_t counter = 0;
static volatile bool motor_running = false;

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

static void IRAM_ATTR button_isr(void *arg){
    uint64_t now = esp_timer_get_time();
    if (now - last_isr_time > DEBOUNCE_DELAY_US){
        motor_running = !motor_running;

        // signal the task that a valid toggle occured
        BaseType_t higher_priority_task_woken = pdFALSE;
        xQueueSendFromISR(button_queue, (void *)&motor_running, &higher_priority_task_woken);

        last_isr_time = now;
        if (higher_priority_task_woken){
            portYIELD_FROM_ISR();
        }
    }
}
void intr_button_init(){
    ESP_LOGI(TAG, "Initializing button and Interrupt");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // rising edge interrupt
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_NUM_INPUT),
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_INPUT, button_isr, NULL);
    ESP_LOGI(TAG, "Button initialization complete");

}

void driver_init(){
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_RST) | (1ULL << PIN_NUM_SLEEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&conf);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1));  
    gpio_set_level(PIN_NUM_RST, 0);  

    spi_bus_config_t bus_conf = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t dev_conf = {
        .clock_speed_hz = 2*1000*1000, // clock out at 2 MHz
        .mode=0,
        .spics_io_num=CS1,
        .flags=SPI_DEVICE_POSITIVE_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_conf, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_conf, &spi_handle));
    gpio_set_level(PIN_NUM_SLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // send mandatory initial SPI settings to enable the driver
    drv8711_write_register(0x00, 0x0C11);


    gpio_set_direction(PIN_NUM_STEP, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_STEP, 0);
    gpio_set_direction(PIN_NUM_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_DIR, 0); // set direction
}

void step(){
    gpio_set_level(PIN_NUM_STEP, 1);
    esp_rom_delay_us(3);

    gpio_set_level(PIN_NUM_STEP, 0);
    esp_rom_delay_us(3);
}
void setDirection(bool dir){
    esp_rom_delay_us(1);
    gpio_set_level(PIN_NUM_DIR, dir);
    esp_rom_delay_us(1);
}
void stepper_task(void *pvParameters){
    bool local_motor_state = false;

    while(1){
        

        if (xQueueReceive(button_queue, &local_motor_state, 0) == pdTRUE){
            ESP_LOGI(TAG, "Motor state changed to %s", local_motor_state ? "RUNNING" : "STOPPED");
        }

        if(local_motor_state){
            step();
            esp_rom_delay_us(STEP_PERIOD_US - 6);
            vTaskDelay(1);
        } else {
            // block/sleep when motor is off
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void stepper_task_no_button(void *pvParameters){
    //bool local_motor_state = false;

    while(1){
        /*if (xQueueReceive(button_queue, &local_motor_state, 0) == pdTRUE){
            ESP_LOGI(TAG, "Motor state changed to %s", local_motor_state ? "RUNNING" : "STOPPED");
        }*/
        printf("running task \n");
        step();
        esp_rom_delay_us(STEP_PERIOD_US - 6);
        vTaskDelay(1);
            // block/sleep when motor is off
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    button_queue = xQueueCreate(10, sizeof(bool));

    driver_init();
    intr_button_init();
    setDirection(1);

    printf("Toggle button on GPIO %d", PIN_NUM_INPUT);
    xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);
}
