/* AS7262 Light Sensor Test

*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "AS7262.h"


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 

static const char *TAG = "as7262-test";

i2c_port_t i2c_num = I2C_MASTER_NUM;

/******** I2C ********/

// ESP I2C Driver setup 
esp_err_t i2c_master_driver_initialize(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);   
}

/**
 * @brief generic function for reading I2C data
 * 
 * @param reg_addr register adress to read from 
 * @param reg_data pointer to save the data read 
 * @param len length of data to be read
 * @param intf_ptr 
 * 
 * >init: dev->intf_ptr = &dev_addr;
 * 
 * @return ESP_OK/BME280_OK if reading was successful
 */
int8_t main_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { // *intf_ptr = dev->intf_ptr
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    uint8_t addr = *(uint8_t*)intf_ptr;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    rslt = ret;
    return rslt;

}

/**
 * @brief generic function for writing data via I2C 
 *  
 * @param reg_addr register adress to write to 
 * @param reg_data register data to be written 
 * @param len length of data to be written
 * @param intf_ptr 
 * 
 * @return ESP_OK/BME280_OK if writing was successful
 */
int8_t main_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    // for (int i = 0; i < len; i++) {
    //     i2c_master_write_byte(cmd, reg_data[i], ACK_CHECK_EN);
    // }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    rslt = ret;
    return rslt;

}

static void main_task(void *arg) {
    ESP_LOGI(TAG, "AS7262 main task initializing...");
    // i2c_master_init();
    i2c_master_driver_initialize();
    as7262_init(main_i2c_read, main_i2c_write);
    
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
