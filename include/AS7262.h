/**
 * @file AS7262.h
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

#ifndef __AS7262_H__
#define __AS7262_H__

#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <string.h>


#define AS7262_ADDRESS (0x49)  /**< Default device I2C address */

#define AS7262_INTEGRATION_TIME_MULT 2.8    /**< Multiplier for integration time */
#define AS7262_NUM_CHANNELS 6               /**< Number of sensor channels */


/**
 * @brief  Virtual register addresses
 */
enum as7262_virtual_registers {
    AS726X_HW_VERSION = 0x00,
    AS726X_FW_VERSION = 0x02,
    AS726X_CONTROL_SETUP = 0x04,
    AS726X_INT_T = 0x05,
    AS726X_DEVICE_TEMP = 0x06,
    AS726X_LED_CONTROL = 0x07,

    // for reading sensor data
    AS7262_V_HIGH = 0x08,
    AS7262_V_LOW = 0x09,
    AS7262_B_HIGH = 0x0A,
    AS7262_B_LOW = 0x0B,
    AS7262_G_HIGH = 0x0C,
    AS7262_G_LOW = 0x0D,
    AS7262_Y_HIGH = 0x0E,
    AS7262_Y_LOW = 0x0F,
    AS7262_O_HIGH = 0x10,
    AS7262_O_LOW = 0x11,
    AS7262_R_HIGH = 0x12,
    AS7262_R_LOW = 0x13,

    AS7262_V_CAL = 0x14,
    AS7262_B_CAL = 0x18,
    AS7262_G_CAL = 0x1C,
    AS7262_Y_CAL = 0x20,
    AS7262_O_CAL = 0x24,
    AS7262_R_CAL = 0x28
};

/**
 *  @brief  Hardware registers addresses
 */
enum as7262_hardware_registers {
    AS726X_SLAVE_STATUS_REG = 0x00,
    AS726X_SLAVE_WRITE_REG = 0x01,
    AS726X_SLAVE_READ_REG = 0x02,
    AS726X_SLAVE_TX_VALID = 0x02,
    AS726X_SLAVE_RX_VALID = 0x01,
};

/**
 * @brief  Color register addresses 
 */
enum as7262_color_registers {
    AS7262_VIOLET = 0x08,
    AS7262_BLUE = 0x0A,
    AS7262_GREEN = 0x0C,
    AS7262_YELLOW = 0x0E,
    AS7262_ORANGE = 0x10,
    AS7262_RED = 0x12,
    AS7262_VIOLET_CALIBRATED = 0x14,
    AS7262_BLUE_CALIBRATED = 0x18,
    AS7262_GREEN_CALIBRATED = 0x1C,
    AS7262_YELLOW_CALIBRATED = 0x20,
    AS7262_ORANGE_CALIBRATED = 0x24,
    AS7262_RED_CALIBRATED = 0x28,
};

/**
 * @brief  conversion modes. Default is Mode 2
 */
enum conversion_types {
    MODE_0 = 0b00,
    MODE_1 = 0b01,
    MODE_2 = 0b10, // default
    ONE_SHOT = 0b11,
};

/**
 * @brief gain settings. Default is 1x gain
 */
enum channel_gain {
    GAIN_1X = 0b00, // default
    GAIN_3X7 = 0b01,
    GAIN_16X = 0b10,
    GAIN_64X = 0b11,
};

/**
 * @brief  indicator LED current limit settings. Default is 1mA
 */
enum ind_led_current_limits {
    LIMIT_1MA = 0b00, // default
    LIMIT_2MA = 0b01,
    LIMIT_4MA = 0b10,
    LIMIT_8MA = 0b11,
};

/**
 * @brief  Driver LED current limit settings. Default is 12.5 mA
 */
enum drv_led_current_limits {
    LIMIT_12MA5 = 0b00, // default
    LIMIT_25MA = 0b01,
    LIMIT_50MA = 0b10,
    LIMIT_100MA = 0b11,
};


/**
 * @brief  Color definitions used by the library
 */
/**************************************************************************/
enum as7262_colors {
  AS726x_VIOLET = 0,
  AS726x_BLUE,
  AS726x_GREEN,
  AS726x_YELLOW,
  AS726x_ORANGE,
  AS726x_RED,
};

/*** Function Pointers ***/
typedef int8_t (*as7262_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

typedef int8_t (*as7262_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);




// esp_err_t as7262_i2c_master_read(uint8_t address, uint8_t *data_rd, size_t size);
// esp_err_t as7262_i2c_master_write(uint8_t address, uint8_t *data_wr, size_t size);


/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with AS726x
   spectral sensors
*/
/**************************************************************************/

typedef struct _as7262_control_setup {
    uint8_t unused;

    /* 1: data ready to be read, sets int active if int is enabled */
    uint8_t DATA_RDY;

    /* conversion type
     *  0b00 = Mode 0
     *  0b01 = Mode 1
     *  0b10 = Mode 2
     *  0b11 = Mode 3 One shot
     */
    uint8_t BANK;

    /* Channel gain setting (all channels)
     *  0b00 = 1x
     *  0b01 = 3.7x
     *  0b10 = 16x
     *  0b11 = 64x
     */
    uint8_t GAIN;

    /* enable or disable interrupt */
    uint8_t INT;
    
    uint8_t RST;
} as7262_control_setup_t;

typedef struct _as7262_int_time {
    uint8_t INT_T;  /**< Integration time (multiplied by INTEGRATION_TIME_MULT) in ms */
} as7262_int_time_t;

typedef struct _as7262_led_control {
    uint8_t LED_IND;  /**< Enable indicator LED (1:Enabled; 0:Disabled) */
    uint8_t ICL_IND;  /**< Indicator LED current limit */

    uint8_t LED_DRV;  /**< Enable LED driver (1:Enabled; 0:Disabled) */
    uint8_t ICL_DRV;  /**< LED driver current limit */
} as7262_led_control_t;


typedef struct as7262_dev {
    /*< Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr;

    /*< I2C Driver Read function pointer */
    as7262_read_fptr_t i2c_read;

    /*< I2C Driver Write function pointer */
    as7262_write_fptr_t i2c_write;

    as7262_control_setup_t control_setup;

    as7262_int_time_t int_time;

    as7262_led_control_t led_control;

    uint16_t raw_values[AS7262_NUM_CHANNELS];

    float calibrated_values[AS7262_NUM_CHANNELS];
} as7262_dev_t;

// as7262_control_setup_t control_setup;  /**! Global AS7262 Control Setup Variable */
// as7262_int_time_t int_time;            /**! Global AS7262 INT TIME Variable */
// as7262_led_control_t led_control;      /**! Global AS7262 LED Control Variable */


/**
 * @brief AS7262 device initialization
 * 
 * @param device            AS7262 device pointer
 * @param user_i2c_read     I2C Driver read function
 * @param user_i2c_write    I2C Driver write function
 * 
 */
void as7262_init(as7262_dev_t *device, as7262_read_fptr_t user_i2c_read, as7262_write_fptr_t user_i2c_write);



/******** Indicator LED (green) interaction functions ************/
/** 
 * @brief Set indicator LED current limit
 * 
 * @param current  LED current limit
 */
void as7262_set_indicator_led_current(as7262_dev_t *device, uint8_t current);

/** 
 * @brief Set indicator LED ON/OFF 
 * 
 * @param on  Boolean: true -> on, false -> off
 */
void as7262_set_indicator_led_on(as7262_dev_t *device, bool on);


/********** LED Driver (bright led) interaction functions ************/

/** 
 * @brief Set White LED current limit
 * 
 * @param current  LED current limit
 */
void as7262_set_led_drv_current(as7262_dev_t *device, uint8_t current);

/** 
 * @brief Set White LED ON/OFF 
 * 
 * @param on  Boolean: true -> on, false -> off
 */
void as7262_set_led_drv_on(as7262_dev_t *device, bool on);



/**
 *  @brief  Set the conversion mode.
 *
 *  @param type the mode to set the sensor to. Should be one of MODE_0, MODE_1,
 *  MODE_2, ONE_SHOT.
 */
void as7262_set_conversion_type(as7262_dev_t *device, uint8_t type);

/**
 *   @brief  Set the sensor gain.
 *   @param gain the gain to set the sensor to. Should be one of GAIN_1X,
 *   GAIN_3X7, GAIN_16X, or GAIN_64X = 0b11.
 */
void as7262_set_gain(as7262_dev_t *device, uint8_t gain);

/**
 *  @brief  Set the integration time for the sensor.
 *  @param time the integration time to set. The actual integration time will be
 *  time*2.8ms
 */
void as7262_set_integration_time(as7262_dev_t *device, uint8_t time);


/**
 *  @brief  enable the device interrupt
 * 
 * @param enabled  Boolean: enabled/disabled
 */
void as7262_enable_interrupt(as7262_dev_t *device, bool enabled);

// /**
//  *  @brief  disable the device interrupt
//  */
// void disable_interrupt();


/** 
 * @brief Read internal device temperature
 * 
 * @return 8-bit device temperature
 */
uint8_t as7262_read_temperature(as7262_dev_t *device);


/** 
 * @brief Indicates if there is data to be read
 * 
 * @return bool: true -> data ready
 */
bool as7262_data_ready(as7262_dev_t *device);





/**
 *  @brief  begin a measurement. This sets the conversion mode to ONE_SHOT.
 */
void as7262_start_single_measurement(as7262_dev_t *device);


/**
 *  @brief  Read an individual raw spectral channel
 * 
 *  @param channel the channel to read
 * 
 *  @return the reading as a raw 16-bit integer
 */
uint16_t as7262_read_channel(as7262_dev_t *device, uint8_t channel);

/**
 *   @brief  Read the raw channels, stores it in device->raw_values
 *
 *   @param num Optional number of channels to read. Defaults to
 *  AS726x_NUM_CHANNELS
*/
void as7262_read_raw_values(as7262_dev_t *device, uint8_t num);

/**
 *   @brief  read an individual calibrated spectral channel
 *   @param channel the channel to read
 *   @return the reading as a raw 16-bit integer
*/
float as7262_read_single_channel_calibrated_value(as7262_dev_t *device, uint8_t channel);

/**
 *  @brief  Read the calibrated values, stores it in device->calibrated_values
 * 
 *  @param num Optional number of channels to read. Defaults to
 *  AS726x_NUM_CHANNELS
*/
void as7262_read_calibrated_values(as7262_dev_t *device, uint8_t num);



/*********** Virtual Registers Read/Write ***********/






#endif  // __AS7262_H__