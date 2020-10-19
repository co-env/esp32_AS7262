/* AS7262 Light Sensor Test

*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "AS7262.h"

static const char *TAG = "as7262-test";

static void main_task(void *arg) {
    ESP_LOGI(TAG, "AS7262 main task initializing...");
    // i2c_master_init();
    as7262_init();
    
    vTaskDelay(1000 / portTICK_RATE_MS);

    set_led_drv_on(true);
    vTaskDelay(1000 / portTICK_RATE_MS);
    set_led_drv_on(false);

    uint8_t temp = read_temperature();

    float sensorValues[AS7262_NUM_CHANNELS];

    // start_measurement();
    set_conversion_type(MODE_2);

    // Changing GAIN 
    control_setup.GAIN = GAIN_16X;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));

    while(1) {
        if (data_ready()) {
            read_calibrated_values(sensorValues, AS7262_NUM_CHANNELS);
            temp = read_temperature();

            ESP_LOGI(TAG, "Device temperature: %d", temp);
            ESP_LOGI(TAG, " Violet:  %f", sensorValues[AS726x_VIOLET]);
            ESP_LOGI(TAG, " Blue:  %f", sensorValues[AS726x_BLUE]);
            ESP_LOGI(TAG, " Green:  %f", sensorValues[AS726x_GREEN]);
            ESP_LOGI(TAG, " Yellow:  %f", sensorValues[AS726x_YELLOW]);
            ESP_LOGI(TAG, " Orange:  %f", sensorValues[AS726x_ORANGE]);
            ESP_LOGI(TAG, " Red:  %f", sensorValues[AS726x_RED]);
            ESP_LOGI(TAG, " ------------------ ");

        }

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    
    // Creation of main task
    xTaskCreate(main_task, "i2c_main_test_task", 1024 * 2, (void *)0, 10, NULL);
}
