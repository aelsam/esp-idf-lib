/**
 * @file BMI270.h
 * @defgroup BMI270 BMI270
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
#ifndef __BMI270_H__
#define __BMI270_H__

#include <stdbool.h>
#include <esp_err.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI270_ADDR       0x36 //!< I2C device address with ADDR pin connected to ground

#define BMI270_MAX_VALUE 0x7fff //!< Maximum ADC value


#endif /* __BMI270_H__ */
