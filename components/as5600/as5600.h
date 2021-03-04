/**
 * @file as5600.h
 * @defgroup as5600 as5600
 * @{
 *
 * ESP-IDF driver for ADS1113/ADS1114/ADS1115, ADS1013/ADS1014/ADS1015 I2C ADC
 *
 * Ported from esp-open-rtos
 *
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (C) 2020 Lucio Tarantino (https://github.com/dianlight)
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_ADDR       0x36 //!< I2C device address with ADDR pin connected to ground

#define as5600_MAX_VALUE 0x7fff //!< Maximum ADC value
#define ADS101X_MAX_VALUE 0x7ff

// ADS101X overrides
#define ADS101X_DATA_RATE_128  	as5600_DATA_RATE_8
#define ADS101X_DATA_RATE_250  	as5600_DATA_RATE_16
#define ADS101X_DATA_RATE_490  	as5600_DATA_RATE_32
#define ADS101X_DATA_RATE_920  	as5600_DATA_RATE_64
#define ADS101X_DATA_RATE_1600	as5600_DATA_RATE_128
#define ADS101X_DATA_RATE_2400	as5600_DATA_RATE_250
#define ADS101X_DATA_RATE_3300	as5600_DATA_RATE_475

/**
 * Gain amplifier
 */
typedef enum
{
    as5600_GAIN_6V144 = 0, //!< +-6.144V
    as5600_GAIN_4V096,     //!< +-4.096V
    as5600_GAIN_2V048,     //!< +-2.048V (default)
    as5600_GAIN_1V024,     //!< +-1.024V
    as5600_GAIN_0V512,     //!< +-0.512V
    as5600_GAIN_0V256,     //!< +-0.256V
    as5600_GAIN_0V256_2,   //!< +-0.256V (same as as5600_GAIN_0V256)
    as5600_GAIN_0V256_3,   //!< +-0.256V (same as as5600_GAIN_0V256)
} as5600_gain_t;

/**
 * Gain amplifier values
 */
extern const float as5600_gain_values[];

/**
 * Input multiplexer configuration (ADS1115 only)
 */
typedef enum
{
    as5600_MUX_0_1 = 0, //!< positive = AIN0, negative = AIN1 (default)
    as5600_MUX_0_3,     //!< positive = AIN0, negative = AIN3
    as5600_MUX_1_3,     //!< positive = AIN1, negative = AIN3
    as5600_MUX_2_3,     //!< positive = AIN2, negative = AIN3
    as5600_MUX_0_GND,   //!< positive = AIN0, negative = GND
    as5600_MUX_1_GND,   //!< positive = AIN1, negative = GND
    as5600_MUX_2_GND,   //!< positive = AIN2, negative = GND
    as5600_MUX_3_GND,   //!< positive = AIN3, negative = GND
} as5600_mux_t;

/**
 * Data rate
 */
typedef enum
{
    as5600_DATA_RATE_8 = 0, //!< 8 samples per second
    as5600_DATA_RATE_16,    //!< 16 samples per second
    as5600_DATA_RATE_32,    //!< 32 samples per second
    as5600_DATA_RATE_64,    //!< 64 samples per second
    as5600_DATA_RATE_128,   //!< 128 samples per second (default)
    as5600_DATA_RATE_250,   //!< 250 samples per second
    as5600_DATA_RATE_475,   //!< 475 samples per second
    as5600_DATA_RATE_860    //!< 860 samples per second
} as5600_data_rate_t;

/**
 * Operational mode
 */
typedef enum
{
    as5600_MODE_CONTINUOS = 0, //!< Continuous conversion mode
    as5600_MODE_SINGLE_SHOT    //!< Power-down single-shot mode (default)
} as5600_mode_t;

/**
 * Comparator mode (ADS1114 and ADS1115 only)
 */
typedef enum
{
    as5600_COMP_MODE_NORMAL = 0, //!< Traditional comparator with hysteresis (default)
    as5600_COMP_MODE_WINDOW      //!< Window comparator
} as5600_comp_mode_t;

/**
 * Comparator polarity (ADS1114 and ADS1115 only)
 */
typedef enum
{
    as5600_COMP_POLARITY_LOW = 0, //!< Active low (default)
    as5600_COMP_POLARITY_HIGH     //!< Active high
} as5600_comp_polarity_t;

/**
 * Comparator latch (ADS1114 and ADS1115 only)
 */
typedef enum
{
    as5600_COMP_LATCH_DISABLED = 0, //!< Non-latching comparator (default)
    as5600_COMP_LATCH_ENABLED       //!< Latching comparator
} as5600_comp_latch_t;

/**
 * Comparator queue
 */
typedef enum
{
    as5600_COMP_QUEUE_1 = 0,   //!< Assert ALERT/RDY pin after one conversion
    as5600_COMP_QUEUE_2,       //!< Assert ALERT/RDY pin after two conversions
    as5600_COMP_QUEUE_4,       //!< Assert ALERT/RDY pin after four conversions
    as5600_COMP_QUEUE_DISABLED //!< Disable comparator (default)
} as5600_comp_queue_t;

/**
 * Initialize device descriptior
 * @param dev Device descriptor
 * @param addr Device address
 * @param port I2C port number
 * @param sda_gpio GPIO pin for SDA
 * @param scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
esp_err_t as5600_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port,
        gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * @param dev Pointer to device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t as5600_free_desc(i2c_dev_t *dev);

/**
 * Get device operational status
 * @param dev Device descriptor
 * @param busy `true` when device performing conversion
 * @return `ESP_OK` on success
 */
esp_err_t as5600_is_busy(i2c_dev_t *dev, bool *busy);

/**
 * Begin a single conversion (when in single-shot mode)
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t as5600_start_conversion(i2c_dev_t *dev);

/**
 * Read last conversion result
 * @param dev Device descriptor
 * @param value Last conversion result
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_value(i2c_dev_t *dev, int16_t *value);

/**
 * Read last conversion result for ADS101x
 * @param dev Device descriptor
 * @param value Last conversion result
 * @return `ESP_OK` on success
 */
esp_err_t ads101x_get_value(i2c_dev_t *dev, int16_t *value);

/**
 * Read the programmable gain amplifier configuration
 * (ADS1114 and ADS1115 only).
 * Use `as5600_gain_values[]` for real voltage.
 * @param dev Device descriptor
 * @param gain Gain value
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_gain(i2c_dev_t *dev, as5600_gain_t *gain);

/**
 * Configure the programmable gain amplifier (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param gain Gain value
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_gain(i2c_dev_t *dev, as5600_gain_t gain);

/**
 * Read the input multiplexer configuration (ADS1115 only)
 * @param dev Device descriptor
 * @param mux Input multiplexer configuration
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_input_mux(i2c_dev_t *dev, as5600_mux_t *mux);

/**
 * Configure the input multiplexer configuration (ADS1115 only)
 * @param dev Device descriptor
 * @param mux Input multiplexer configuration
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_input_mux(i2c_dev_t *dev, as5600_mux_t mux);

/**
 * Read the device operating mode
 * @param dev Device descriptor
 * @param mode Device operating mode
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_mode(i2c_dev_t *dev, as5600_mode_t *mode);

/**
 * Set the device operating mode
 * @param dev Device descriptor
 * @param mode Device operating mode
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_mode(i2c_dev_t *dev, as5600_mode_t mode);

/**
 * Read the data rate
 * @param dev Device descriptor
 * @param rate Data rate
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_data_rate(i2c_dev_t *dev, as5600_data_rate_t *rate);

/**
 * Configure the data rate
 * @param dev Device descriptor
 * @param rate Data rate
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_data_rate(i2c_dev_t *dev, as5600_data_rate_t rate);

/**
 * Get comparator mode (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param mode Comparator mode
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_comp_mode(i2c_dev_t *dev, as5600_comp_mode_t *mode);

/**
 * Set comparator mode (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param mode Comparator mode
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_comp_mode(i2c_dev_t *dev, as5600_comp_mode_t mode);

/**
 * Get polarity of the comparator output pin ALERT/RDY
 * (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param polarity Comparator output pin polarity
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_comp_polarity(i2c_dev_t *dev, as5600_comp_polarity_t *polarity);

/**
 * Set polarity of the comparator output pin ALERT/RDY
 * (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param polarity Comparator output pin polarity
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_comp_polarity(i2c_dev_t *dev, as5600_comp_polarity_t polarity);

/**
 * Get comparator output latch mode, see datasheet.
 * (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param latch Comparator output latch mode
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_comp_latch(i2c_dev_t *dev, as5600_comp_latch_t *latch);

/**
 * Set comparator output latch mode (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param latch Comparator output latch mode
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_comp_latch(i2c_dev_t *dev, as5600_comp_latch_t latch);

/**
 * Set number of the comparator conversions before pin ALERT/RDY
 * assertion, or disable comparator (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param queue Number of the comparator conversions
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_comp_queue(i2c_dev_t *dev, as5600_comp_queue_t *queue);

/**
 * Get number of the comparator conversions before pin ALERT/RDY
 * assertion (ADS1114 and ADS1115 only)
 * @param dev Device descriptor
 * @param queue Number of the comparator conversions
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_comp_queue(i2c_dev_t *dev, as5600_comp_queue_t queue);

/**
 * Get the lower threshold value used by comparator
 * @param dev Device descriptor
 * @param th Lower threshold value
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_comp_low_thresh(i2c_dev_t *dev, int16_t *th);

/**
 * Set the lower threshold value used by comparator
 * @param dev Device descriptor
 * @param th Lower threshold value
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_comp_low_thresh(i2c_dev_t *dev, int16_t th);

/**
 * Get the upper threshold value used by comparator
 * @param dev Device descriptor
 * @param th Upper threshold value
 * @return `ESP_OK` on success
 */
esp_err_t as5600_get_comp_high_thresh(i2c_dev_t *dev, int16_t *th);

/**
 * Set the upper threshold value used by comparator
 * @param dev Device descriptor
 * @param th Upper threshold value
 * @return `ESP_OK` on success
 */
esp_err_t as5600_set_comp_high_thresh(i2c_dev_t *dev, int16_t th);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __AS5600_H__ */
