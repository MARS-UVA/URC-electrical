#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

//Define GPIO for i2c peripheral
#define SDA_PIN_W 	GPIO_NUM_9
#define SCL_PIN_W	GPIO_NUM_10

//max_m10s i2c device address
#define I2C_ADD 0x42
#define NUM_BYTES_REGISTER 0xFD
#define DATA_REGISTER 0xFF
typedef struct {
    int utc_hour;
    int utc_minute;
    float utc_second;
    float latitude;
    char lat_N_S;
    float longitude;
    char lon_E_W;
    int fix_quality;
    int num_satellites;
    float horizontal_dilution;
    float altitude;
    char alt_unit;
    float height_geoid;
    char height_geoid_unit;
} NMEAGGA_Data_s;

//Define NMEA GGA data structure
NMEAGGA_Data_s GGAdata;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t gnss_dev;

void parseNMEA_GGA (const char* nmeaGgaString, NMEAGGA_Data_s* GGA_data){
    
    sscanf(nmeaGgaString,"$GNGGA,%2d%2d%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f,%c",
           &GGA_data->utc_hour, &GGA_data->utc_minute, &GGA_data->utc_second,
           &GGA_data->latitude, &GGA_data->lat_N_S,
           &GGA_data->longitude, &GGA_data->lon_E_W,
           &GGA_data->fix_quality, &GGA_data->num_satellites, &GGA_data->horizontal_dilution,
           &GGA_data->altitude, &GGA_data->alt_unit, &GGA_data->height_geoid,
           &GGA_data->height_geoid_unit);
    
    //Adjust the latitude and longitude to decimal representation
    GGA_data->latitude = (int)(GGA_data->latitude / 100) +fmod(GGA_data->latitude, 100) /60.0;
    if(GGA_data->lat_N_S == 'S') GGA_data->latitude = -GGA_data->latitude;
    
    GGA_data->longitude = (int)(GGA_data->longitude / 100) +fmod(GGA_data->longitude, 100) /60.0;
    if(GGA_data->lon_E_W== 'W') GGA_data->longitude = -GGA_data->longitude;

}

//Print on terminal the GGA data 
void printGGA_data(NMEAGGA_Data_s GGA_data){

	printf("UTC: %d:%d:%.0f\n",GGA_data.utc_hour, GGA_data.utc_minute, GGA_data.utc_second);
	printf("Latitude: %f  Longitude: %f  Altitude: %.2f m\n",GGA_data.latitude, GGA_data.longitude, GGA_data.altitude);
	printf("Fix: %d  N of satellites: %d\n",GGA_data.fix_quality, GGA_data.num_satellites);
	printf("Horizontal dilution: %.2f  Height geoid: %.2f %c\n\n", GGA_data.horizontal_dilution, GGA_data.height_geoid, GGA_data.height_geoid_unit);

}


char calculateChecksum(const char* inString) {
    char checksum = 0;
    //Discard the first character '$' - Counts up to the character '*'
    for (const char* p = inString+ 1; *p != '*' && *p != '\0'; p++) {
        checksum ^= *p;   //NMEA use XOR for checksum calculation
    }
    return checksum;
}

// Function to validate the checksum
bool validateChecksum(const char* nmeaString) {
    // Find the position of the '*' character
    const char* checksumPos= strchr(nmeaString, '*');
    if (checksumPos == NULL || checksumPos- nmeaString< 3) {
        return false;  // No '*' found or invalid string length
    }

    // Calculate the checksum
    char calculatedChecksum= calculateChecksum(nmeaString);

    // Extract the checksum from the string
    unsigned int receivedChecksum;
    sscanf(checksumPos + 1, "%2x", &receivedChecksum);

    // Compare the calculated checksum with the received checksum
    return calculatedChecksum == receivedChecksum;
}

/*void i2c_config(void){

	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN_W,
		.scl_io_num = SCL_PIN_W,
		.sda_pullup_en = GPIO_PULLUP_DISABLE,
		.scl_pullup_en = GPIO_PULLUP_DISABLE,
		.master.clk_speed = 400000,
	};
	i2c_param_config(I2C_NUM_0, &conf);

	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}*/


void i2c_config(void){
    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1,
        .sda_io_num = SDA_PIN_W,
        .scl_io_num = SCL_PIN_W,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };


    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADD,
        .scl_speed_hz = 400000,
    };


    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &gnss_dev));
	
}


static void GNSS_manager_task(void *arg){
	uint8_t rx_data[16];
	uint8_t tx_data[16];
	char gps_string_in[250];     // 250 is just a reasonably big number for a NMEA sentence
	int string_idx = 0;
	uint32_t num_to_read = 0;

    uint8_t curReg = NUM_BYTES_REGISTER;
	while(1){
		//i2c_master_write_read_device(I2C_NUM_0, I2C_ADD, tx_data, 1, rx_data, 3, 	
				     //500/portTICK_PERIOD_MS);
        if(i2c_master_transmit_receive(gnss_dev, &curReg, 1, rx_data, 2, 1000) != ESP_OK){
             vTaskDelay(pdMS_TO_TICKS(100));
        }
		num_to_read = (uint32_t)rx_data[1] | ((uint32_t)rx_data[0] << 8);
        while(num_to_read > 0){
            size_t n;
            if (num_to_read > sizeof(rx_data)) {
                n = sizeof(rx_data);
            } else {
                n = num_to_read;
            }
            //i2c_master_write_read_device(I2C_NUM_0, I2C_ADD , tx_data, 1, rx_data, 3, 	
                        //500/portTICK_PERIOD_MS);
            i2c_master_receive(gnss_dev, rx_data, n, -1);
            for (size_t i = 0; i < n; i++) {
                uint8_t cur = rx_data[i];
                if (cur == 0xFF) {
                    continue;
                }          // "no data" filler

                if (cur == '\n') {
                    gps_string_in[string_idx] = '\0';
                    if (strstr(gps_string_in, "GNGGA") != NULL && validateChecksum(gps_string_in)) {
                        parseNMEA_GGA(gps_string_in, &GGAdata);
                        printGGA_data(GGAdata);
                    }
                    string_idx = 0;
                } else if (cur != '\r') {
                    if (string_idx < (int)sizeof(gps_string_in) - 1) gps_string_in[string_idx++] = (char)cur;
                    else string_idx = 0;                  // overflow: drop the line
                }
            }
            num_to_read -= n;
          }
          vTaskDelay(1000/portTICK_PERIOD_MS);
	}

}


void app_main (void)
{
	i2c_config();

	xTaskCreate(GNSS_manager_task, "GNSS_manager", 4*1024, NULL, 5, NULL);

    while(1) {
    	vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
