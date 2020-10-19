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

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */

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
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


// ESP I2C Driver setup 
static esp_err_t i2c_master_driver_initialize(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    return i2c_param_config(i2c_master_port, &conf);
}

/** 
 * @brief Função genérica de leitura i2c de [size] bytes no [address]
 * 
 */

esp_err_t as7262_i2c_master_read(uint8_t address, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = AS7262_ADDRESS;
    uint8_t data_addr = address;
    uint8_t len = size;

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(i2c_master_driver_initialize());

    // i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, )
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data_rd, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + len - 1, NACK_VAL);
    
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // ESP_LOGI(TAG, "I2C Read Success: %02x", data_rd[0]);
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "I2C Read bus is busy");
    } else {
        ESP_LOGW(TAG, "I2C Read failed");
    }

    i2c_driver_delete(I2C_NUM_0);
    
    return 0;
}



/** 
 * @brief Função genérica de escrita i2c de [size] bytes no [address]
 * 
 */

esp_err_t as7262_i2c_master_write(uint8_t address, uint8_t *data_wr, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = AS7262_ADDRESS;
    uint8_t data_addr = address;
    uint8_t len = size;

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_ERROR_CHECK(i2c_master_driver_initialize());


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);

    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data_wr[i], ACK_CHECK_EN);
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

     if (ret == ESP_OK) {
        // ESP_LOGI(TAG, "I2C Write Success: %02x", data_wr[0]);
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "I2C Read bus is busy");
    } else {
        ESP_LOGW(TAG, "I2C Read failed");
    }

    i2c_driver_delete(I2C_NUM_0);

    return 0;
}


uint8_t virtualRead(uint8_t virtual_addr) {
    //! @TODO: checar se teve erro nas chamadas de read e write 

    uint8_t status, read_data;
    esp_err_t err;

    // Fica preso nisso até que o STATUS_REG esteja com o TX_VALID
    while (1) {
        // Read slave I²C status to see if the read buffer is ready.
        err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);

        // ESP_LOGI(TAG, "%s First status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            // No inbound TX pending at slave. Okay to write now.
            break;
        }
    }

    // Escreve o virtual_addr normal* no registrador WRITE_REG (0x01) para indicar que queremos lê-lo
    as7262_i2c_master_write(AS726X_SLAVE_WRITE_REG, &virtual_addr, 1);

    while (1) {
        // Read the slave I²C status to see if our read data is available.
        err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        // ESP_LOGI(TAG, "%s Second status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_RX_VALID) != 0) {
            // Read data is ready
            break;
        }
    }
    
    err = as7262_i2c_master_read(AS726X_SLAVE_READ_REG, &read_data, 1);

    return read_data;
}


void virtualWrite(uint8_t virtual_addr, uint8_t value) {
    //! @TODO: checar se teve erro nas chamadas de read e write

    uint8_t status;
    esp_err_t err;

    // Fica preso nisso até que o STATUS_REG esteja com o TX_VALID
    while (1) {
        // Read slave I²C status to see if the write buffer is ready.
        err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        // ESP_LOGI(TAG, "%s First status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            // No inbound TX pending at slave. Okay to write now.
            break;
        }
    }

    // Escreve o (virtual_addr | 0x80) no registrador WRITE_REG (0x01) para indicar que queremos escrever nesse addr
    uint8_t _addr[1] = { virtual_addr | 0x80 };
    err = as7262_i2c_master_write(AS726X_SLAVE_WRITE_REG, _addr, 1);

    while (1) {
        // Read the slave I²C status to see if the write buffer is ready.
        err = as7262_i2c_master_read(AS726X_SLAVE_STATUS_REG, &status, 1);
        // ESP_LOGI(TAG, "%s Second status reg: %02x", __FUNCTION__ ,status);
        if ((status & AS726X_SLAVE_TX_VALID) == 0) {
            // Read data is ready
            break;
        }
    }

    err = as7262_i2c_master_write(AS726X_SLAVE_WRITE_REG, &value, 1);
}


uint8_t get_control_setup_hex(as7262_control_setup_t s) {
    return ((s.DATA_RDY << 1) | (s.BANK << 2) | (s.GAIN << 4) | (s.INT << 6) | (s.RST << 7));
}

uint8_t get_led_control_hex(as7262_led_control_t led) {
    return (led.LED_IND | (led.ICL_IND << 1) | (led.LED_DRV << 3) | (led.ICL_DRV << 4));
};


void as7262_init(void) {

    ESP_LOGI(TAG, "Hello!");

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







