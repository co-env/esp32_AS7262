/**
 * @file AS7262.c
 *
 * @author Renato Freitas 
 * 
 * @brief AS7262 library, based on Adafruit's one. 
 * ----> https://www.adafruit.com/products/3779
 * ----> https://github.com/adafruit/Adafruit_AS726x
 *
 * @date 10/2020
 */

#include "AS7262.h"

static const char *TAG = "AS7262-LIB";

struct AS7262_dev dev;

uint8_t virtualRead(uint8_t virtual_addr) {
    //! @TODO: checar se teve erro nas chamadas de read e write 

    uint8_t status, read_data;
    esp_err_t err;

    // Fica preso nisso até que o STATUS_REG esteja com o TX_VALID
    while (1) {
        // Read slave I²C status to see if the read buffer is ready.
        // err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        err = dev.i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, dev.intf_ptr);
        if(err == ESP_OK) ESP_LOGI(TAG, "Read complete");
        else ESP_LOGI(TAG, "Could not read");

        // ESP_LOGI(TAG, "%s First status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            // No inbound TX pending at slave. Okay to write now.
            break;
        }
    }

    // Escreve o virtual_addr normal* no registrador WRITE_REG (0x01) para indicar que queremos lê-lo
    // as7262_i2c_master_write(AS726X_SLAVE_WRITE_REG, &virtual_addr, 1);
    err = dev.i2c_write(AS726X_SLAVE_WRITE_REG, &virtual_addr, 1, dev.intf_ptr);
    if(err == ESP_OK) ESP_LOGI(TAG, "Write complete");
    else ESP_LOGI(TAG, "Could not write");

    while (1) {
        // Read the slave I²C status to see if our read data is available.
        // err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        err = dev.i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, dev.intf_ptr);

        // ESP_LOGI(TAG, "%s Second status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_RX_VALID) != 0) {
            // Read data is ready
            break;
        }
    }
    
    // err = as7262_i2c_master_read(AS726X_SLAVE_READ_REG, &read_data, 1);
    err = dev.i2c_read(AS726X_SLAVE_READ_REG, &read_data, 1, dev.intf_ptr);

    return read_data;
}


void virtualWrite(uint8_t virtual_addr, uint8_t value) {
    //TODO: checar se teve erro nas chamadas de read e write

    uint8_t status;
    esp_err_t err;

    // Fica preso nisso até que o STATUS_REG esteja com o TX_VALID
    while (1) {
        // Read slave I²C status to see if the write buffer is ready.
        // err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        err = dev.i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, dev.intf_ptr);
        
        // ESP_LOGI(TAG, "%s First status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            // No inbound TX pending at slave. Okay to write now.
            break;
        }
    }

    // Escreve o (virtual_addr | 0x80) no registrador WRITE_REG (0x01) para indicar que queremos escrever nesse addr
    uint8_t _addr[1] = { virtual_addr | 0x80 };
    // err = as7262_i2c_master_write(AS726X_SLAVE_WRITE_REG, _addr, 1);
    err = dev.i2c_write(AS726X_SLAVE_WRITE_REG, _addr, 1, dev.intf_ptr);

    while (1) {
        // Read the slave I²C status to see if the write buffer is ready.
        // err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        err = dev.i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, dev.intf_ptr);
        
        // ESP_LOGI(TAG, "%s Second status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            // Read data is ready
            break;
        }
    }

    // err = as7262_i2c_master_write(AS726X_SLAVE_WRITE_REG, &value, 1);
    err = dev.i2c_write(AS726X_SLAVE_WRITE_REG, &value, 1, dev.intf_ptr);

}


uint8_t get_control_setup_hex(as7262_control_setup_t s) {
    return ((s.DATA_RDY << 1) | (s.BANK << 2) | (s.GAIN << 4) | (s.INT << 6) | (s.RST << 7));
}

uint8_t get_led_control_hex(as7262_led_control_t led) {
    return (led.LED_IND | (led.ICL_IND << 1) | (led.LED_DRV << 3) | (led.ICL_DRV << 4));
};


void as7262_init(as7262_read_fptr_t user_i2c_read, as7262_write_fptr_t user_i2c_write) {

    ESP_LOGI(TAG, "Hello!");
    uint8_t dev_addr = AS7262_ADDRESS;
    dev.intf_ptr = &dev_addr; //??
    dev.i2c_read = user_i2c_read;
    dev.i2c_write = user_i2c_write;

    // Soft reset do CI
    control_setup.RST = 1;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
    control_setup.RST = 0;

    vTaskDelay(1000 / portTICK_RATE_MS);

    uint8_t version = virtualRead(AS726X_HW_VERSION);
    if (version != 0x40) {
        ESP_LOGW(TAG, "%s -- Wrong Hardware Version: %02x", __FUNCTION__, version);
    } else {
        ESP_LOGI(TAG, "%s -- Hardware Version: %02x", __FUNCTION__, version);
        ESP_LOGI(TAG, "%s -- AS7262 Sensor Initialized", __FUNCTION__);
        set_indicator_led_on(true);
    }
}

void set_indicator_led_current(uint8_t current) {
    led_control.ICL_IND = current;
    virtualWrite(AS726X_LED_CONTROL, get_led_control_hex(led_control));
}

void set_indicator_led_on(bool on) {
    led_control.LED_IND = on ? 1 : 0;
    virtualWrite(AS726X_LED_CONTROL, get_led_control_hex(led_control));
    ESP_LOGD(TAG, "Indicator LED Changed!");
}

void set_led_drv_current(uint8_t current) {
    led_control.LED_IND = current;
    virtualWrite(AS726X_LED_CONTROL, get_led_control_hex(led_control));
}

void set_led_drv_on(bool on) {
    led_control.LED_DRV = on ? 1 : 0;
    virtualWrite(AS726X_LED_CONTROL, get_led_control_hex(led_control));
}

uint8_t read_temperature() {
    return (virtualRead(AS726X_DEVICE_TEMP));
}

bool data_ready() {
    return (virtualRead(AS726X_CONTROL_SETUP) & 0x02);
}


void set_conversion_type(uint8_t type) {
    control_setup.BANK = type;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
}

void set_gain(uint8_t gain) {
    control_setup.GAIN = gain;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
}

void set_integration_time(uint8_t time) {
    int_time.INT_T = time;
    virtualWrite(AS726X_INT_T, int_time.INT_T);
}

void enable_interrupt() {
    control_setup.INT = 1;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
}

void disable_interrupt() {
    control_setup.INT = 0;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
}

void start_measurement() {
    control_setup.DATA_RDY = 0;
    virtualWrite(AS726X_CONTROL_SETUP, get_control_setup_hex(control_setup));
    
    set_conversion_type(ONE_SHOT);
}

uint16_t read_channel(uint8_t channel) {
    return (virtualRead(channel) << 8) | virtualRead(channel + 1);
}

void read_raw_values(uint16_t *buf, uint8_t num) {
    for (int i = 0; i < num; i++) {
        switch (i) {
            case AS726x_VIOLET:
                buf[i] = read_channel(AS7262_VIOLET);
            break;

            case AS726x_BLUE:
                buf[i] = read_channel(AS7262_BLUE);
            break;

            case AS726x_GREEN:
                buf[i] = read_channel(AS7262_GREEN);
            break;

            case AS726x_YELLOW:
                buf[i] = read_channel(AS7262_YELLOW);
            break;

            case AS726x_ORANGE:
                buf[i] = read_channel(AS7262_ORANGE);
            break;

            case AS726x_RED:
                buf[i] = read_channel(AS7262_RED);
            break;

            default:
            break;
        }
    }
}

float read_calibrated_value(uint8_t channel) {
    uint32_t val = 0;
    val =   ((uint32_t)virtualRead(channel) << 24) |
            ((uint32_t)virtualRead(channel + 1) << 16) |
            ((uint32_t)virtualRead(channel + 2) << 8) |
            (uint32_t)virtualRead(channel + 3);

    float ret;
    memcpy(&ret, &val, 4);
    return ret;
}

void read_calibrated_values(float *buf, uint8_t num) {
    for (int i = 0; i < num; i++) {
        switch (i) {
            case AS726x_VIOLET:
                buf[i] = read_calibrated_value(AS7262_VIOLET_CALIBRATED);
            break;

            case AS726x_BLUE:
                buf[i] = read_calibrated_value(AS7262_BLUE_CALIBRATED);
            break;

            case AS726x_GREEN:
                buf[i] = read_calibrated_value(AS7262_GREEN_CALIBRATED);
            break;

            case AS726x_YELLOW:
                buf[i] = read_calibrated_value(AS7262_YELLOW_CALIBRATED);
            break;

            case AS726x_ORANGE:
                buf[i] = read_calibrated_value(AS7262_ORANGE_CALIBRATED);
            break;

            case AS726x_RED:
                buf[i] = read_calibrated_value(AS7262_RED_CALIBRATED);
            break;

            default:
            break;
        }
    }
}







