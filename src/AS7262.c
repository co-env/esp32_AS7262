/**
 * @file AS7262.c
 *
 * @author Renato Freitas
 * @author Isabella Bologna
 * 
 * @brief AS7262 library, based on Adafruit's one. 
 * ----> https://www.adafruit.com/products/3779
 * ----> https://github.com/adafruit/Adafruit_AS726x
 *
 * @date 10/2020
 */

#include "AS7262.h"

#include "esp_log.h"

static const char *TAG = "AS7262-LIB";

static uint8_t dev_addr = AS7262_ADDRESS;


/*******************************************
 ****** Private Functions Prototypes ******
 *******************************************/

/**
 * @brief Read virtual register address
 * @param virtual_addr  Virtual register address
 */
static uint8_t as7262_virtual_read(as7262_dev_t *device, uint8_t virtual_addr);

/**
 * @brief Write virtual register address
 * @param virtual_addr  Virtual register address
 * @param value  Value to write
 */
static void as7262_virtual_write(as7262_dev_t *device, uint8_t virtual_addr, uint8_t value);

/**
 * @brief Joins control_setup on a single byte
 * 
 * @param s  Control setup struct
 * 
 * @return Single byte control setup
 */
static uint8_t get_control_setup_hex(as7262_control_setup_t s);


/**
 * @brief Joins led_control on a single byte
 * 
 * @param led  Led Control struct
 * 
 * @return Single byte led control
 */
static uint8_t get_led_control_hex(as7262_led_control_t led);




/*******************************************
 ****** Public Functions Definitions *******
 *******************************************/


void as7262_init(as7262_dev_t *device, as7262_read_fptr_t user_i2c_read, as7262_write_fptr_t user_i2c_write) {

    ESP_LOGI(TAG, "Hello!");
    device->intf_ptr = &dev_addr; //??
    device->i2c_read = user_i2c_read;
    device->i2c_write = user_i2c_write;

    // Soft reset do CI
    device->control_setup.RST = 1;
    as7262_virtual_write(device, AS726X_CONTROL_SETUP, get_control_setup_hex(device->control_setup));
    device->control_setup.RST = 0;

    vTaskDelay(1000 / portTICK_RATE_MS);

    uint8_t version = as7262_virtual_read(device, AS726X_HW_VERSION);
    if (version != 0x40) {
        ESP_LOGW(TAG, "%s -- Wrong Hardware Version: %02x", __FUNCTION__, version);
    } else {
        ESP_LOGI(TAG, "%s -- Hardware Version: %02x", __FUNCTION__, version);
        ESP_LOGI(TAG, "%s -- AS7262 Sensor Initialized", __FUNCTION__);
        as7262_set_indicator_led_on(device, true);
    }
}

void as7262_set_indicator_led_current(as7262_dev_t *device, uint8_t current) {
    device->led_control.ICL_IND = current;
    as7262_virtual_write(device, AS726X_LED_CONTROL, get_led_control_hex(device->led_control));
}

void as7262_set_indicator_led_on(as7262_dev_t *device, bool on) {
    device->led_control.LED_IND = on ? 1 : 0;
    as7262_virtual_write(device, AS726X_LED_CONTROL, get_led_control_hex(device->led_control));
    
    ESP_LOGD(TAG, "Indicator LED Changed!");
}

void as7262_set_led_drv_current(as7262_dev_t *device, uint8_t current) {
    device->led_control.LED_IND = current;
    as7262_virtual_write(device, AS726X_LED_CONTROL, get_led_control_hex(device->led_control));
}

void as7262_set_led_drv_on(as7262_dev_t *device, bool on) {
    device->led_control.LED_DRV = on ? 1 : 0;
    as7262_virtual_write(device, AS726X_LED_CONTROL, get_led_control_hex(device->led_control));
}

void as7262_set_conversion_type(as7262_dev_t *device, uint8_t type) {
    device->control_setup.BANK = type;
    as7262_virtual_write(device, AS726X_CONTROL_SETUP, get_control_setup_hex(device->control_setup));
}

void as7262_set_gain(as7262_dev_t *device, uint8_t gain) {
    device->control_setup.GAIN = gain;
    as7262_virtual_write(device, AS726X_CONTROL_SETUP, get_control_setup_hex(device->control_setup));
}

void as7262_set_integration_time(as7262_dev_t *device, uint8_t time) {
    device->int_time.INT_T = time;
    as7262_virtual_write(device, AS726X_INT_T, device->int_time.INT_T);
}

void as7262_enable_interrupt(as7262_dev_t *device, bool enabled) {
    device->control_setup.INT = enabled ? 1 : 0;
    as7262_virtual_write(device, AS726X_CONTROL_SETUP, get_control_setup_hex(device->control_setup));
}

uint8_t as7262_read_temperature(as7262_dev_t *device) {
    return (as7262_virtual_read(device, AS726X_DEVICE_TEMP));
}

bool as7262_data_ready(as7262_dev_t *device) {
    return (as7262_virtual_read(device, AS726X_CONTROL_SETUP) & 0x02);
}


void as7262_start_single_measurement(as7262_dev_t *device) {
    device->control_setup.DATA_RDY = 0;
    as7262_virtual_write(device, AS726X_CONTROL_SETUP, get_control_setup_hex(device->control_setup));
    
    as7262_set_conversion_type(device, ONE_SHOT);
}

uint16_t as7262_read_channel(as7262_dev_t *device, uint8_t channel) {
    return (as7262_virtual_read(device, channel) << 8) | as7262_virtual_read(device, channel + 1);
}

void as7262_read_raw_values(as7262_dev_t *device, uint8_t num) {
    for (int i = 0; i < num; i++) {
        switch (i) {
            case AS726x_VIOLET:
                device->raw_values[i] = as7262_read_channel(device, AS7262_VIOLET);
            break;

            case AS726x_BLUE:
                device->raw_values[i] = as7262_read_channel(device, AS7262_BLUE);
            break;

            case AS726x_GREEN:
                device->raw_values[i] = as7262_read_channel(device, AS7262_GREEN);
            break;

            case AS726x_YELLOW:
                device->raw_values[i] = as7262_read_channel(device, AS7262_YELLOW);
            break;

            case AS726x_ORANGE:
                device->raw_values[i] = as7262_read_channel(device, AS7262_ORANGE);
            break;

            case AS726x_RED:
                device->raw_values[i] = as7262_read_channel(device, AS7262_RED);
            break;

            default:
            break;
        }
    }
}

float as7262_read_calibrated_value(as7262_dev_t *device, uint8_t channel) {
    uint32_t val = 0;
    val =   ((uint32_t)as7262_virtual_read(device, channel) << 24) |
            ((uint32_t)as7262_virtual_read(device, channel + 1) << 16) |
            ((uint32_t)as7262_virtual_read(device, channel + 2) << 8) |
            (uint32_t)as7262_virtual_read(device, channel + 3);

    float ret;
    memcpy(&ret, &val, 4);
    return ret;
}

void as7262_read_calibrated_values(as7262_dev_t *device, uint8_t num) {
    for (int i = 0; i < num; i++) {
        switch (i) {
            case AS726x_VIOLET:
                device->calibrated_values[i] = as7262_read_calibrated_value(device, AS7262_VIOLET_CALIBRATED);
            break;

            case AS726x_BLUE:
                device->calibrated_values[i] = as7262_read_calibrated_value(device, AS7262_BLUE_CALIBRATED);
            break;

            case AS726x_GREEN:
                device->calibrated_values[i] = as7262_read_calibrated_value(device, AS7262_GREEN_CALIBRATED);
            break;

            case AS726x_YELLOW:
                device->calibrated_values[i] = as7262_read_calibrated_value(device, AS7262_YELLOW_CALIBRATED);
            break;

            case AS726x_ORANGE:
                device->calibrated_values[i] = as7262_read_calibrated_value(device, AS7262_ORANGE_CALIBRATED);
            break;

            case AS726x_RED:
                device->calibrated_values[i] = as7262_read_calibrated_value(device, AS7262_RED_CALIBRATED);
            break;

            default:
            break;
        }
    }
}



/*******************************************
 ****** Private Functions Definitions ******
 *******************************************/

static uint8_t as7262_virtual_read(as7262_dev_t *device, uint8_t virtual_addr) {
    //TODO: checar se teve erro nas chamadas de read e write 

    uint8_t status, read_data;
    esp_err_t err;

    // Fica preso nisso até que o STATUS_REG esteja com o TX_VALID
    //! Adicionar um timeout????
    while (1) {
        // Read slave I²C status to see if the read buffer is ready.
        err = device->i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, device->intf_ptr);
        
        if (err == ESP_OK) 
            ESP_LOGD(TAG, "Read complete");
        else 
            ESP_LOGD(TAG, "Could not read");

        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            break;  // No inbound TX pending at slave. Okay to write now.
        }
    }

    // Escreve o virtual_addr normal* no registrador WRITE_REG (0x01) para indicar que queremos lê-lo
    err = device->i2c_write(AS726X_SLAVE_WRITE_REG, &virtual_addr, 1, device->intf_ptr);
    
    if(err == ESP_OK)
        ESP_LOGD(TAG, "Write complete");
    else
        ESP_LOGD(TAG, "Could not write");

    while (1) {
        // Read the slave I²C status to see if our read data is available.
        err = device->i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, device->intf_ptr);

        if ((status & AS726X_SLAVE_RX_VALID) != 0) {
            break;  // Read data is ready
        }
    }
    
    err = device->i2c_read(AS726X_SLAVE_READ_REG, &read_data, 1, device->intf_ptr);

    return read_data;
}


static void as7262_virtual_write(as7262_dev_t *device, uint8_t virtual_addr, uint8_t value) {
    //TODO: checar se teve erro nas chamadas de read e write

    uint8_t status;
    esp_err_t err;

    // Fica preso nisso até que o STATUS_REG esteja com o TX_VALID
    //! Adicionar um timeout????
    while (1) {
        // Read slave I²C status to see if the write buffer is ready.
        err = device->i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, device->intf_ptr);
        
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            break;  // No inbound TX pending at slave. Okay to write now.
        }
    }

    // Writes (virtual_addr | 0x80) WRITE_REG (0x01) register to indicate write attempt on this virtual addr
    uint8_t _addr[1] = { virtual_addr | 0x80 };
    
    err = device->i2c_write(AS726X_SLAVE_WRITE_REG, _addr, 1, device->intf_ptr);

    while (1) {
        // Read the slave I²C status to see if the write buffer is ready.
        err = device->i2c_read(AS726X_SLAVE_STATUS_REG, &status, 1, device->intf_ptr);
        
        // ESP_LOGI(TAG, "%s Second status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            break;  // No inbound TX pending at slave. Okay to write now.
        }
    }

    err = device->i2c_write(AS726X_SLAVE_WRITE_REG, &value, 1, device->intf_ptr);
}

static uint8_t get_control_setup_hex(as7262_control_setup_t s) {
    return ((s.DATA_RDY << 1) | (s.BANK << 2) | (s.GAIN << 4) | (s.INT << 6) | (s.RST << 7));
}

static uint8_t get_led_control_hex(as7262_led_control_t led) {
    return (led.LED_IND | (led.ICL_IND << 1) | (led.LED_DRV << 3) | (led.ICL_DRV << 4));
};
