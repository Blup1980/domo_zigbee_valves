/*
 * Simple valve driver using GPIO outputs.
 * Active-high polarity assumed (GPIO=1 opens valve).
 * GPIO mappings are defined in valve_driver.h; negative values are treated as unconfigured.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "valve_driver.h"

static const char *TAG = "valve_driver";

static int s_valve_gpios[VALVE_COUNT];

/* runtime state */
static valve_state_t s_valve_state[VALVE_COUNT];
static TimerHandle_t  s_valve_timer[VALVE_COUNT]; /* timer used to finish opening after VALVE_OPENING_MS */
static SemaphoreHandle_t s_lock;

/* Helper: count valves in OPENING state (no separate counter to avoid sync issues) */
static int count_opening(void)
{
    int c = 0;
    for (int i = 0; i < VALVE_COUNT; ++i) {
        if (s_valve_state[i] == VALVE_STATE_OPENING) ++c;
    }
    return c;
}



/* Per-valve pending state: a valve can be VALVE_STATE_PENDING when an open
 * request arrived but capacity was full. When a slot is available the driver
 * will select the lowest-index valve with VALVE_STATE_PENDING and start it.
 */
static bool find_lowest_pending(uint8_t *out_idx)
{
    for (uint8_t i = 0; i < VALVE_COUNT; ++i) {
        if (s_valve_state[i] == VALVE_STATE_PENDING) {
            *out_idx = i;
            return true;
        }
    }
    return false;
}

/* Requires s_lock held. */
static void pending_enqueue_locked(uint8_t idx)
{
    /* mark valve as pending if it's currently closed (avoid duplicates) */
    if (idx >= VALVE_COUNT) return;
    if (s_valve_state[idx] == VALVE_STATE_CLOSED) {
        s_valve_state[idx] = VALVE_STATE_PENDING;
        /* notify application (LED/status indication, etc.) */
        valve_changed_callback(idx, VALVE_STATE_PENDING);
    }
}

/* Start (or restart) the one-shot opening timer. Requires s_lock held. */
static esp_err_t arm_opening_timer_locked(uint8_t idx)
{
    if (idx >= VALVE_COUNT || s_valve_timer[idx] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Best-effort stop; timer might already be stopped. */
    (void)xTimerStop(s_valve_timer[idx], 0);
    if (xTimerChangePeriod(s_valve_timer[idx], pdMS_TO_TICKS(VALVE_OPENING_MS), 0) != pdPASS) {
        return ESP_FAIL;
    }
    if (xTimerStart(s_valve_timer[idx], 0) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Try to start exactly one pending valve (lowest index). Requires s_lock held. */
static void try_start_one_pending_locked(void)
{
    while (count_opening() < VALVE_MAX_CONCURRENT_OPENING) {
        uint8_t next_idx;
        if (!find_lowest_pending(&next_idx)) {
            return;
        }

        int next_gpio = s_valve_gpios[next_idx];
        if (next_gpio < 0) {
            /* Misconfiguration: can't actuate. Clear pending so it doesn't block the queue. */
            ESP_LOGW(TAG, "Valve %u pending but GPIO not configured; dropping request", (unsigned)(next_idx + 1));
            s_valve_state[next_idx] = VALVE_STATE_CLOSED;
            valve_changed_callback(next_idx, VALVE_STATE_CLOSED);
            continue;
        }

        esp_err_t err = gpio_set_level(next_gpio, 1);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Valve %u: gpio_set_level(%d,1) failed: %s", (unsigned)(next_idx + 1), next_gpio, esp_err_to_name(err));
            /* Keep it pending; caller can retry via later transitions. */
            return;
        }

        s_valve_state[next_idx] = VALVE_STATE_OPENING;
        if (arm_opening_timer_locked(next_idx) != ESP_OK) {
            ESP_LOGW(TAG, "Valve %u: failed to arm opening timer", (unsigned)(next_idx + 1));
            /* Roll back to pending so it can be retried. */
            s_valve_state[next_idx] = VALVE_STATE_PENDING;
            return;
        }

        ESP_LOGI(TAG, "Started opening pending valve %u. Opening count=%d", (unsigned)(next_idx + 1), count_opening());
        valve_changed_callback(next_idx, VALVE_STATE_OPENING);
        return;
    }
}


static void finish_opening(TimerHandle_t timer)
{
    uintptr_t idx_raw = (uintptr_t)pvTimerGetTimerID(timer);
    if (idx_raw >= VALVE_COUNT) return;
    uint8_t idx = (uint8_t)idx_raw;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* If the valve is no longer in OPENING state (eg. it was closed and the timer
     * was stopped/deleted), ignore this callback. */
    if (s_valve_state[idx] != VALVE_STATE_OPENING) {
        ESP_LOGI(TAG, "finish_opening: valve %d not OPENING (state=%d), ignoring", idx + 1, s_valve_state[idx]);
        xSemaphoreGive(s_lock);
        return;
    }

    int gpio = s_valve_gpios[idx];
    if (gpio >= 0) {
        esp_err_t err = gpio_set_level(gpio, 1); /* ensure open */
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Valve %u: gpio_set_level(%d,1) failed: %s", (unsigned)(idx + 1), gpio, esp_err_to_name(err));
        }
    }
    s_valve_state[idx] = VALVE_STATE_OPEN;
    ESP_LOGI(TAG, "Valve %d finished opening (now OPEN). Opening count=%d", idx + 1, count_opening());

    /* start next pending if any (choose lowest index) */
    try_start_one_pending_locked();
    valve_changed_callback(idx, VALVE_STATE_OPEN);
    xSemaphoreGive(s_lock);
}

static void populate_gpio_table(void)
{
    /* Map macros into table */
    s_valve_gpios[0]  = VALVE_GPIO_0;
    s_valve_gpios[1]  = VALVE_GPIO_1;
    s_valve_gpios[2]  = VALVE_GPIO_2;
    s_valve_gpios[3]  = VALVE_GPIO_3;
    s_valve_gpios[4]  = VALVE_GPIO_4;
    s_valve_gpios[5]  = VALVE_GPIO_5;
    s_valve_gpios[6]  = VALVE_GPIO_6;
    s_valve_gpios[7]  = VALVE_GPIO_7;
    s_valve_gpios[8]  = VALVE_GPIO_8;
    s_valve_gpios[9]  = VALVE_GPIO_9;
    s_valve_gpios[10] = VALVE_GPIO_10;
}

esp_err_t valve_driver_init(bool default_open)
{
    populate_gpio_table();

    /* initialize sequencing primitives first (callbacks may call back into the driver) */
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    for (int i = 0; i < VALVE_COUNT; ++i) {
        int gpio = s_valve_gpios[i];
        s_valve_state[i] = default_open ? VALVE_STATE_OPEN : VALVE_STATE_CLOSED;

        /* Create one timer per valve; we stop/start it as needed (no deletes during runtime). */
        s_valve_timer[i] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)i), finish_opening);
        if (s_valve_timer[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create timer for valve %d", i + 1);
            return ESP_ERR_NO_MEM;
        }

        if (gpio < 0) {
            ESP_LOGW(TAG, "Valve %d: GPIO not configured", i + 1);
            valve_changed_callback(i, default_open ? VALVE_STATE_OPEN : VALVE_STATE_CLOSED);
            continue;
        }

        io_conf.pin_bit_mask = 1ULL << gpio;
        esp_err_t err = gpio_config(&io_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_config failed for gpio %d: %d", gpio, err);
            return err;
        }

        /* Active-high polarity: true -> set level 1 */
        int level = default_open ? 1 : 0;
        err = gpio_set_level(gpio, level);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_set_level failed for gpio %d: %d", gpio, err);
            return err;
        }
        ESP_LOGI(TAG, "Valve %d (GPIO %d) initialized -> %s", i + 1, gpio, level ? "OPEN" : "CLOSED");
        valve_changed_callback(i, default_open ? VALVE_STATE_OPEN : VALVE_STATE_CLOSED);
    }

    return ESP_OK;
}

esp_err_t valve_driver_set_power(uint8_t valve_index, bool on)
{
    if (valve_index >= VALVE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    int gpio = s_valve_gpios[valve_index];
    if (gpio < 0) {
        ESP_LOGW(TAG, "Valve %d: GPIO not configured, cannot set state", valve_index + 1);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!on) {
        /* Close immediately (instantaneous) and cancel any opening */
        bool was_opening = (s_valve_state[valve_index] == VALVE_STATE_OPENING);
        /* remove pending state if set */
        if (s_valve_state[valve_index] == VALVE_STATE_PENDING) {
            s_valve_state[valve_index] = VALVE_STATE_CLOSED;
        }
        if (s_valve_timer[valve_index] != NULL) {
            (void)xTimerStop(s_valve_timer[valve_index], 0);
        }
        /* If the valve was opening
         * then we should attempt to start a queued valve to fill the freed slot.
         */
        /* set it to closed so we don't count it as OPENING anymore */
        s_valve_state[valve_index] = VALVE_STATE_CLOSED;

        esp_err_t err = gpio_set_level(gpio, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Valve %u: gpio_set_level(%d,0) failed: %s", (unsigned)(valve_index + 1), gpio, esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "Valve %d -> CLOSED (immediate). Opening count=%d", valve_index + 1, count_opening());
        /* Report physical state: 1 = Closed */
        valve_changed_callback(valve_index, VALVE_STATE_CLOSED);

        if (was_opening) {
            /* Closing an opening valve frees a slot: start one queued valve if any. */
            try_start_one_pending_locked();
        }

        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* Request to open: if already open or opening, ignore */
    if (s_valve_state[valve_index] == VALVE_STATE_OPEN || s_valve_state[valve_index] == VALVE_STATE_OPENING) {
        ESP_LOGI(TAG, "Valve %d already OPEN/OPENING, ignoring open request", valve_index + 1);
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* If we have capacity, start opening immediately (count via states) */
    if (count_opening() < VALVE_MAX_CONCURRENT_OPENING) {
        esp_err_t err = gpio_set_level(gpio, 1); /* start power-hungry transition */
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Valve %u: gpio_set_level(%d,1) failed: %s", (unsigned)(valve_index + 1), gpio, esp_err_to_name(err));
            xSemaphoreGive(s_lock);
            return err;
        }
        s_valve_state[valve_index] = VALVE_STATE_OPENING;
        /* Report physical state: 2 = Opening */
        valve_changed_callback(valve_index, VALVE_STATE_OPENING);
        /* start timer to finish opening */
        if (arm_opening_timer_locked(valve_index) != ESP_OK) {
            /* Roll back to CLOSED to avoid getting stuck in OPENING without a timer. */
            ESP_LOGW(TAG, "Valve %u: failed to arm opening timer", (unsigned)(valve_index + 1));
            s_valve_state[valve_index] = VALVE_STATE_CLOSED;
            (void)gpio_set_level(gpio, 0);
            valve_changed_callback(valve_index, VALVE_STATE_CLOSED);
            xSemaphoreGive(s_lock);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Started opening valve %d. Opening count=%d", valve_index + 1, count_opening());
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* Otherwise enqueue request and return success; HA sees immediate attribute accept */
    pending_enqueue_locked(valve_index);
    ESP_LOGI(TAG, "Valve %d open request queued (concurrent limit reached).", valve_index + 1);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

void __attribute__((weak)) valve_changed_callback(uint8_t valve_index, valve_state_t new_state)
{
    /* weak callback to be optionally implemented by the application for state change notifications */
    ESP_LOGI(TAG, "Valve %d state changed callback: new state=%d", valve_index + 1, new_state);
}
