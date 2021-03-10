// Copyright 2020 Espressif Systems (Shanghai) Co. Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _IOT_BUTTON_V2_H_
#define _IOT_BUTTON_V2_H_

#include "button_adc.h"
#include "button_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (* button_v2_cb_t)(void *);
typedef void *button_v2_handle_t;

/**
 * @brief Button events
 *
 */
typedef enum {
    BUTTON_PRESS_DOWN = 0,
    BUTTON_PRESS_UP,
    BUTTON_PRESS_REPEAT,
    BUTTON_SINGLE_CLICK,
    BUTTON_DOUBLE_CLICK,
    BUTTON_LONG_PRESS_START,
    BUTTON_LONG_PRESS_HOLD,
    BUTTON_EVENT_MAX,
    BUTTON_NONE_PRESS,
} button_v2_event_t;

/**
 * @brief Supported button type
 *
 */
typedef enum {
    BUTTON_TYPE_GPIO,
    BUTTON_TYPE_ADC,
} button_v2_type_t;

/**
 * @brief Button configuration
 *
 */
typedef struct {
    button_v2_type_t type;                           /**< button type, The corresponding button configuration must be filled */
    union {
        button_gpio_config_t gpio_button_config; /**< gpio button configuration */
        button_adc_config_t adc_button_config;   /**< adc button configuration */
    }; /**< button configuration */
} button_v2_config_t;

/**
 * @brief Create a button
 *
 * @param config pointer of button configuration, must corresponding the button type
 *
 * @return A handle to the created button, or NULL in case of error.
 */
button_v2_handle_t iot_button_v2_create(const button_v2_config_t *config);

/**
 * @brief Delete a button
 *
 * @param btn_handle A button handle to delete
 *
 * @return
 *      - ESP_OK  Success
 *      - ESP_FAIL Failure
 */
esp_err_t iot_button_v2_delete(button_v2_handle_t btn_handle);

/**
 * @brief Register the button event callback function.
 *
 * @param btn_handle A button handle to register
 * @param event Button event
 * @param cb Callback function.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG   Arguments is invalid.
 */
esp_err_t iot_button_v2_register_cb(button_v2_handle_t btn_handle, button_v2_event_t event, button_v2_cb_t cb);

/**
 * @brief Unregister the button event callback function.
 *
 * @param btn_handle A button handle to unregister
 * @param event Button event
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG   Arguments is invalid.
 */
esp_err_t iot_button_v2_unregister_cb(button_v2_handle_t btn_handle, button_v2_event_t event);

/**
 * @brief Get button event
 *
 * @param btn_handle Button handle
 *
 * @return Current button event. See button_event_t
 */
button_v2_event_t iot_button_v2_get_event(button_v2_handle_t btn_handle);

/**
 * @brief Get button repeat times
 *
 * @param btn_handle Button handle
 *
 * @return button pressed times. For example, double-click return 2, triple-click return 3, etc.
 */
uint8_t iot_button_v2_get_repeat(button_v2_handle_t btn_handle);

#ifdef __cplusplus
}
#endif

#endif