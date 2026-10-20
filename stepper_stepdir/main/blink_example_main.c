#include <stdint.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define AIN1_GPIO   4
#define AIN2_GPIO   5
#define BIN1_GPIO   6
#define BIN2_GPIO   7
#define STBY_GPIO   8

#define POT_ADC_CHANNEL ADC_CHANNEL_0 // GPIO 1 on ESP32-s3

#define MAX_ADC 4095
#define CENTER_ADC 2048
#define CENTER_BAND 150

#define MIN_DELAY_MS 2
#define MAX_DELAY_MS 20
#define STEP_DELAY_MS 5
static const char *TAG = "STEPPER";

static const uint8_t sequence[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1}
};

static int phase = 0;
static adc_oneshot_unit_handle_t adc_handle;

static void set_coils(int index){
    gpio_set_level(AIN1_GPIO, sequence[index][0]);
    gpio_set_level(AIN2_GPIO, sequence[index][1]);
    gpio_set_level(BIN1_GPIO, sequence[index][2]);
    gpio_set_level(BIN2_GPIO, sequence[index][3]);

}

static void stepper_init(void){
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << AIN1_GPIO | 1ULL << AIN2_GPIO | 1ULL << BIN1_GPIO | 1ULL << BIN2_GPIO | 1ULL << STBY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(STBY_GPIO, 0); // when STBY is low, driver is disabled
    set_coils(phase); // set_coils(0)
    gpio_set_level(STBY_GPIO, 1); // enable driver
}

static void potentiometer_init(void){
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &channel_cfg));
}

static int read_potentiometer(void){
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &raw));

    return raw;
}

static void stepper_step(int direction){
    phase = (phase + direction + 4) & 4;
    set_coils(phase);
}

static void stepper_move(int steps)
{
    int direction = (steps >= 0) ? 1 : -1;
    int count = (steps >= 0) ? steps : -steps;

    for (int i = 0; i < count; i++) {
        phase = (phase + direction + 4) % 4;

        set_coils(phase);

        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
    }
}


void app_main(void)
{
    stepper_init();
    // potentiometer_init();

    while (1) {
        /*
        int reading = read_potentiometer();
        int offset = reading - CENTER_ADC;

        if (offset > CENTER_BAND || offset < -CENTER_BAND) {
            int magnitude = offset >= 0 ? offset : -offset;

            // Map knob displacement to a delay:
            // farther from center = shorter delay = faster motor.
            int delay_ms =
                MAX_DELAY_MS -
                (magnitude * (MAX_DELAY_MS - MIN_DELAY_MS)
                 / CENTER_ADC);

            if (delay_ms < MIN_DELAY_MS) {
                delay_ms = MIN_DELAY_MS;
            }

            int direction = offset > 0 ? 1 : -1;
            */
            gpio_set_level(STBY_GPIO, 1); // enable driver

            ESP_LOGI(TAG, "Forward");
            stepper_step(200);
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "backwards");
            stepper_step(-200);
            vTaskDelay(pdMS_TO_TICKS(1000));
            // Disable the driver after the demonstration.
            gpio_set_level(STBY_GPIO, 0);

            ESP_LOGI(TAG, "test complete");

        /*} else {
            // Near the center: stop stepping.
            vTaskDelay(pdMS_TO_TICKS(5));
        }
            */
    }
}