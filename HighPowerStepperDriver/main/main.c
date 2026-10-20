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
#define PIN_NUM_MISO    25
#define PIN_NUM_MOSI    23
#define PIN_NUM_CLK     19
#define CS1             22

#define PIN_NUM_STEP    12
#define PIN_NUM_DIR     11
#define PIN_NUM_SLEEP   14
#define PIN_NUM_FAULT   15
#define PIN_NUM_RST     5

#define PIN_NUM_INPUT   4 // button pin
#define DEBOUNCE_DELAY_US 200000ULL // debounce delay in ms (200ms)

// this period is length of the delay between steps
// increase delay -- slower
// decrease delay -- faster, but there is a limit before 
// it starts skipping steps
#define STEP_PERIOD_US 2000

static const char *TAG = "STEPPER";

static QueueHandle_t button_queue;
static volatile uint64_t last_isr_time = 0;
static volatile uint32_t counter = 0;
static volatile bool motor_running = false;

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

int button_read(){
    return gpio_get_level(PIN_NUM_INPUT);
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
    };
    spi_device_interface_config_t dev_conf = {
        .clock_speed_hz = 2*1000*1000, // clock out at 2 MHz
        .mode=0,
        .spics_io_num=CS1,
        .flags=SPI_DEVICE_POSITIVE_CS
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
void app_main(void)
{
    button_queue = xQueueCreate(10, sizeof(bool));

    driver_init();
    intr_button_init();
    setDirection(0);

    printf("Toggle button on GPIO %d", PIN_NUM_INPUT);
    xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);
}
