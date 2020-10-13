/**
 * @file AS7262.h
 *
 * @author Renato Freitas 
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
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <string.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define AS7262_ADDRESS (0x49)  /**< default I2C address */
/*=========================================================================*/

/**************************************************************************/
/*!
    @brief  virtual registers
*/
/**************************************************************************/
enum {
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

/**************************************************************************/
/*!
    @brief  hardware registers
*/
/**************************************************************************/
enum {
  AS726X_SLAVE_STATUS_REG = 0x00,
  AS726X_SLAVE_WRITE_REG = 0x01,
  AS726X_SLAVE_READ_REG = 0x02,
  AS726X_SLAVE_TX_VALID = 0x02,
  AS726X_SLAVE_RX_VALID = 0x01,
};

/**************************************************************************/
/*!
    @brief  color registers
*/
/**************************************************************************/
enum {
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

/**************************************************************************/
/*!
    @brief  conversion modes. Default is Mode 2
*/
/**************************************************************************/
enum conversion_types {
  MODE_0 = 0b00,
  MODE_1 = 0b01,
  MODE_2 = 0b10, // default
  ONE_SHOT = 0b11,
};

/**************************************************************************/
/*!
    @brief gain settings. Default is 1x gain
*/
/**************************************************************************/
enum channel_gain {
  GAIN_1X = 0b00, // default
  GAIN_3X7 = 0b01,
  GAIN_16X = 0b10,
  GAIN_64X = 0b11,
};

/**************************************************************************/
/*!
    @brief  indicator LED current limit settings. Default is 1mA
*/
/**************************************************************************/
enum ind_led_current_limits {
  LIMIT_1MA = 0b00, // default
  LIMIT_2MA = 0b01,
  LIMIT_4MA = 0b10,
  LIMIT_8MA = 0b11,
};

/**************************************************************************/
/*!
    @brief  Driver LED current limit settings. Default is 12.5 mA
*/
/**************************************************************************/
enum drv_led_current_limits {
  LIMIT_12MA5 = 0b00, // default
  LIMIT_25MA = 0b01,
  LIMIT_50MA = 0b10,
  LIMIT_100MA = 0b11,
};

/*=========================================================================*/

#define AS7262_INTEGRATION_TIME_MULT 2.8 ///< multiplier for integration time
#define AS7262_NUM_CHANNELS 6            ///< number of sensor channels

/**************************************************************************/
/*!
    @brief  Color definitions used by the library
*/
/**************************************************************************/
enum {
  AS726x_VIOLET = 0,
  AS726x_BLUE,
  AS726x_GREEN,
  AS726x_YELLOW,
  AS726x_ORANGE,
  AS726x_RED,
};


esp_err_t as7262_i2c_master_read(uint8_t address, uint8_t *data_rd, size_t size);
esp_err_t as7262_i2c_master_write(uint8_t address, uint8_t *data_wr, size_t size);


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


as7262_control_setup_t control_setup;  /**! Global AS7262 Control Setup Variable */
as7262_int_time_t int_time;            /**! Global AS7262 INT TIME Variable */
as7262_led_control_t led_control;      /**! Global AS7262 LED Control Variable */


uint8_t get_control_setup_hex(as7262_control_setup_t s);
uint8_t get_led_control_hex(as7262_led_control_t led);



/**
 * @brief AS7262 Chip Init
 */
void as7262_init(void);



/******** Indicator LED ************/
/** 
 * @brief Set indicator LED current
 */
void set_indicator_led_current(uint8_t current);

// @brief: turn on/off indicator
void set_indicator_led_on(bool on);


/********** LED Driver ************/
// turn on the drv led
void set_led_drv_on(bool on);
// set current through drv led
void set_led_drv_current(uint8_t current);

uint8_t read_temperature();

bool data_ready();


/**
 *  @brief  Set the conversion mode.
 *  @param type the mode to set the sensor to. Should be one of MODE_0, MODE_1,
 *  MODE_2, ONE_SHOT.
 */
void set_conversion_type(uint8_t type);

/**
 *   @brief  Set the sensor gain.
 *   @param gain the gain to set the sensor to. Should be one of GAIN_1X,
 *   GAIN_3X7, GAIN_16X, or GAIN_64X = 0b11.
 */
void set_gain(uint8_t gain);

/**
 *  @brief  Set the integration time for the sensor.
 *  @param time the integration time to set. The actual integration time will be
 *  time*2.8ms
 */
void set_integration_time(uint8_t time);

/**
 *  @brief  enable the device interrupt
 */
void enable_interrupt();

/**
 *  @brief  disable the device interrupt
 */
void disable_interrupt();

/**
 *  @brief  begin a measurement. This sets the conversion mode to ONE_SHOT.
 */
void start_measurement();

/**
 *   @brief  read an individual raw spectral channel
 *   @param channel the channel to read
 *   @return the reading as a raw 16-bit integer
 */
uint16_t read_channel(uint8_t channel);

/**
 *   @brief  read the raw channels
 *   @param buf the buffer to read the data into
 *   @param num Optional number of channels to read. Defaults to
 *  AS726x_NUM_CHANNELS
*/
void read_raw_values(uint16_t *buf, uint8_t num);

/**
 *   @brief  read an individual calibrated spectral channel
 *   @param channel the channel to read
 *   @return the reading as a raw 16-bit integer
*/
float read_calibrated_value(uint8_t channel);

/**
 *   @brief  read the raw channels
 *   @param buf the buffer to read the data into
 *   @param num Optional number of channels to read. Defaults to
 *  AS726x_NUM_CHANNELS
*/
void read_calibrated_values(float *buf, uint8_t num);



/*********** Virtual Registers Read/Write ***********/

/**
 * @brief Read virtual register address
 * @param virtual_addr  Virtual register address
 */
uint8_t virtualRead(uint8_t virtual_addr);

/**
 * @brief Write virtual register address
 * @param virtual_addr  Virtual register address
 * @param value  Value to write
 */
void virtualWrite(uint8_t virtual_addr, uint8_t value);


#endif  // __AS7262_H__