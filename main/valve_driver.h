/*
 * Valve driver - GPIO mappings
 *
 * Configure the VALVE_GPIO_x defines to the actual GPIO numbers for your board.
 * Any negative value is treated as "not configured" and the driver will refuse
 * to actuate that valve.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VALVE_COUNT 11

/* Placeholder GPIO mapping for valves 0..10 (Valve 1..11).
 * Replace -1 with actual GPIO numbers. Negative values mean "not configured".
 */
#define VALVE_GPIO_0  10
#define VALVE_GPIO_1  11
#define VALVE_GPIO_2  25
#define VALVE_GPIO_3  12
#define VALVE_GPIO_4  22
#define VALVE_GPIO_5  0
#define VALVE_GPIO_6  1
#define VALVE_GPIO_7  2
#define VALVE_GPIO_8  3
#define VALVE_GPIO_9  4
#define VALVE_GPIO_10 5

typedef enum {
    VALVE_STATE_CLOSED = 0,
    VALVE_STATE_OPENING,
    VALVE_STATE_OPEN,
    VALVE_STATE_PENDING
} valve_state_t;

/**
 * @brief Initialize valve driver and configure GPIOs.
 *
 * @param default_open If true set valves open (GPIO=1 for active-high), otherwise closed.
 * @return ESP_OK on success (even if some GPIOs are not configured), error code otherwise.
 */
esp_err_t valve_driver_init(bool default_open);

/**
 * @brief Set a valve state.
 *
 * @param valve_index Index in range [0..VALVE_COUNT-1]
 * @param on true=open, false=closed
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG for bad index or not configured
 */
esp_err_t valve_driver_set_power(uint8_t valve_index, bool on);

void valve_changed_callback(uint8_t valve_index, valve_state_t new_state);

#ifdef __cplusplus
}
#endif
