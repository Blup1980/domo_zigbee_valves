/*
 * Valve driver - placeholder GPIO mappings
 *
 * Configure the VALVE_GPIO_x defines to the actual GPIO numbers for your board.
 * By default they are set to -1 as placeholders. Update them before flashing.
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
#define VALVE_GPIO_0  -1
#define VALVE_GPIO_1  -1
#define VALVE_GPIO_2  -1
#define VALVE_GPIO_3  -1
#define VALVE_GPIO_4  -1
#define VALVE_GPIO_5  -1
#define VALVE_GPIO_6  -1
#define VALVE_GPIO_7  -1
#define VALVE_GPIO_8  -1
#define VALVE_GPIO_9  -1
#define VALVE_GPIO_10 -1

/**
 * @brief Initialize valve driver and configure GPIOs.
 *
 * @param default_open If true set valves open (GPIO=1 for active-high), otherwise closed.
 * @return ESP_OK on success (even if some GPIOs are placeholders), error code otherwise.
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

/* For testing/inspection: get current physical state
 * returns true if physically open (GPIO=1), false otherwise
 */
bool valve_driver_get_physical_state(uint8_t valve_index);

#ifdef __cplusplus
}
#endif
