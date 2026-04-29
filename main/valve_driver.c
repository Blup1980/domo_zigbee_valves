/*
 * Simple valve driver using GPIO outputs.
 * Active-high polarity assumed (GPIO=1 opens valve).
 * Placeholder GPIOs are defined in valve_driver.h; negative values are treated as unconfigured.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "valve_driver.h"
/* Zigbee API for attribute updates and reporting */
#include "esp_zigbee.h"

static const char *TAG = "valve_driver";

static int s_valve_gpios[VALVE_COUNT];

typedef enum { VALVE_STATE_CLOSED = 0, VALVE_STATE_OPENING, VALVE_STATE_OPEN, VALVE_STATE_PENDING } valve_state_t;

/* runtime state */
static valve_state_t s_valve_state[VALVE_COUNT];
static TimerHandle_t  s_valve_timer[VALVE_COUNT]; /* timer used to finish opening after 3 minutes */
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

/* forward declaration so helper functions can call it before its definition */
static void report_multistate_present_value(uint8_t valve_index, uint16_t present_value);

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

/* constants */
#define VALVE_OPENING_MS (3 * 60 * 1000) /* 3 minutes */
#define VALVE_MAX_CONCURRENT_OPENING 4

static void pending_enqueue(uint8_t idx)
{
    /* mark valve as pending if it's currently closed (avoid duplicates) */
    if (idx >= VALVE_COUNT) return;
    if (s_valve_state[idx] == VALVE_STATE_CLOSED) {
        s_valve_state[idx] = VALVE_STATE_PENDING;
        /* report pending as 'Opening' in Multistate Input so HA sees activity */
        report_multistate_present_value(idx, 2);
    }
}

#ifdef CONFIG_UNITY
valve_drv_state_t valve_driver_test_get_state(uint8_t valve_index)
{
    if (valve_index >= VALVE_COUNT) return VALVE_DRV_STATE_CLOSED;
    return (valve_drv_state_t)s_valve_state[valve_index];
}

int valve_driver_test_get_opening_count(void)
{
    return count_opening();
}

int valve_driver_test_get_pending_length(void)
{
    int c = 0;
    for (int i = 0; i < VALVE_COUNT; ++i) if (s_valve_state[i] == VALVE_STATE_PENDING) ++c;
    return c;
}

esp_err_t valve_driver_test_finish_open(uint8_t valve_index)
{
    if (valve_index >= VALVE_COUNT) return ESP_ERR_INVALID_ARG;
    /* call the finish_opening logic directly to simulate timer expiry */
    if (s_valve_timer[valve_index] != NULL) {
        /* For unit tests, the test code may maintain its own timer objects. If so, we can
         * call finish_opening with the stored timer handle. Otherwise, try to call
         * finish_opening directly which uses pvTimerGetTimerID to extract index. */
        finish_opening(s_valve_timer[valve_index]);
        return ESP_OK;
    }
    /* if no timer, still set state to open and report */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_valve_state[valve_index] = VALVE_STATE_OPEN;
    /* start next pending if any (lowest index) */
    uint8_t next_idx;
    if (count_opening() < VALVE_MAX_CONCURRENT_OPENING && find_lowest_pending(&next_idx)) {
        int next_gpio = s_valve_gpios[next_idx];
        if (next_gpio >= 0) gpio_set_level(next_gpio, 1);
        s_valve_state[next_idx] = VALVE_STATE_OPENING;
        if (s_valve_timer[next_idx] == NULL) {
            s_valve_timer[next_idx] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)next_idx), finish_opening);
        }
        xTimerStart(s_valve_timer[next_idx], 0);
        report_multistate_present_value(next_idx, 2);
    }
    xSemaphoreGive(s_lock);
    report_multistate_present_value(valve_index, 3);
    return ESP_OK;
}
#endif

#ifdef CONFIG_UNITY
int valve_driver_test_get_pending_at(int pos)
{
    int len = valve_driver_test_get_pending_length();
    if (pos < 0 || pos >= len) return -1;
    int seen = 0;
    for (int i = 0; i < VALVE_COUNT; ++i) {
        if (s_valve_state[i] == VALVE_STATE_PENDING) {
            if (seen == pos) return i;
            ++seen;
        }
    }
    return -1;
}

int valve_driver_test_get_max_concurrent_opening(void)
{
    return VALVE_MAX_CONCURRENT_OPENING;
}
#endif

/* Helper: send a ZCL report for Multistate Input PresentValue for an endpoint.
 * Mapping: valve_index 0 -> endpoint 11, etc.
 * PresentValue: 1=Closed, 2=Opening, 3=Open
 */
static void report_multistate_present_value(uint8_t valve_index, uint16_t present_value)
{
    uint8_t ep = 11 + valve_index;
    uint16_t cluster_id = EZB_ZCL_CLUSTER_ID_MULTISTATE_INPUT;
    uint16_t attr_id = EZB_ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE_ID;
    /* attribute value is uint16_t */
    uint16_t val = present_value;
    /* The ESP-Zigbee API requires holding the Zigbee lock when calling into the ZCL stack
     * from non-Zigbee threads (timer callbacks, etc.). Acquire the lock before updating
     * attribute and sending report, then release it. */
    if (!esp_zigbee_lock_acquire(portMAX_DELAY)) {
        ESP_LOGW(TAG, "Failed to acquire Zigbee lock to report MultistateInput for ep %d", ep);
        return;
    }

    ezb_zcl_status_t st = ezb_zcl_set_attr_value(ep, cluster_id, EZB_ZCL_CLUSTER_SERVER, attr_id, EZB_ZCL_STD_MANUF_CODE, &val, false);
    if (st != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to set MultistateInput PresentValue for ep %d: %d", ep, st);
    }

    ezb_zcl_report_attr_cmd_t rep = {0};
    rep.cmd_ctrl.src_ep = ep;
    rep.cmd_ctrl.dst_ep = 0;
    rep.cmd_ctrl.cluster_id = cluster_id;
    rep.cmd_ctrl.fc.dis_default_rsp = 1; /* disable default response */
    rep.payload.attr_id = attr_id;
    (void)ezb_zcl_report_attr_cmd_req(&rep);

    esp_zigbee_lock_release();
}

/* In tests we may want to assert certain invariants; provide a debug-only invariant
 * check to help catch logic errors during development. This is a noop in production.
 */
#ifdef CONFIG_UNITY
static void invariant_check(void)
{
    int opening = 0;
    for (int i = 0; i < VALVE_COUNT; ++i) {
        if (s_valve_state[i] == VALVE_STATE_OPENING) ++opening;
    }
    /* opening must not exceed allowed maximum */
    TEST_ASSERT_LESS_OR_EQUAL_INT(VALVE_MAX_CONCURRENT_OPENING, opening);
}
#else
static void invariant_check(void) {}
#endif

static void finish_opening(TimerHandle_t timer)
{
    uint32_t idx = (uint32_t)pvTimerGetTimerID(timer);
    if (idx >= VALVE_COUNT) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* If the valve is no longer in OPENING state (eg. it was closed and the timer
     * was stopped/deleted), ignore this callback. */
    if (s_valve_state[idx] != VALVE_STATE_OPENING) {
        ESP_LOGI(TAG, "finish_opening: valve %d not OPENING (state=%d), ignoring", idx + 1, s_valve_state[idx]);
        xSemaphoreGive(s_lock);
        return;
    }

    int gpio = s_valve_gpios[idx];
    if (gpio >= 0) gpio_set_level(gpio, 1); /* ensure open */
    s_valve_state[idx] = VALVE_STATE_OPEN;
    ESP_LOGI(TAG, "Valve %d finished opening (now OPEN). Opening count=%d", idx + 1, count_opening());

    /* start next pending if any (choose lowest index) */
    uint8_t next_idx;
    if (count_opening() < VALVE_MAX_CONCURRENT_OPENING && find_lowest_pending(&next_idx)) {
        int next_gpio = s_valve_gpios[next_idx];
        if (next_gpio >= 0) gpio_set_level(next_gpio, 1); /* start power-hungry transition */
        s_valve_state[next_idx] = VALVE_STATE_OPENING;
        /* start/arm timer for next_idx */
        if (s_valve_timer[next_idx] == NULL) {
            s_valve_timer[next_idx] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)next_idx), finish_opening);
        }
        xTimerStart(s_valve_timer[next_idx], 0);
        ESP_LOGI(TAG, "Started opening pending valve %d. Opening count=%d", next_idx + 1, count_opening());
        /* report opening state for the started valve */
        report_multistate_present_value(next_idx, 2);
    }

    /* Report physical state change (3-state) for this valve: 3 = Open */
    report_multistate_present_value((uint8_t)idx, 3);

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

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    for (int i = 0; i < VALVE_COUNT; ++i) {
        int gpio = s_valve_gpios[i];
        s_valve_state[i] = default_open ? VALVE_STATE_OPEN : VALVE_STATE_CLOSED;
        s_valve_timer[i] = NULL;
        if (gpio < 0) {
            ESP_LOGW(TAG, "Valve %d: GPIO not configured (placeholder)", i + 1);
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
    }

    /* initialize sequencing primitives */
    s_lock = xSemaphoreCreateMutex();

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
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!on) {
        /* Close immediately (instantaneous) and cancel any opening */
        /* remove pending state if set */
        if (s_valve_state[valve_index] == VALVE_STATE_PENDING) s_valve_state[valve_index] = VALVE_STATE_CLOSED;
        if (s_valve_timer[valve_index] != NULL) {
            xTimerStop(s_valve_timer[valve_index], 0);
            xTimerDelete(s_valve_timer[valve_index], 0);
            s_valve_timer[valve_index] = NULL;
        }
        /* If the valve was opening
         * then we should attempt to start a queued valve to fill the freed slot.
         */
        if (s_valve_state[valve_index] == VALVE_STATE_OPENING) {
            /* set it to closed so we don't count it as in opening state anymore for the total count */
            s_valve_state[valve_index] = VALVE_STATE_CLOSED;
            uint8_t next_idx;
            if (count_opening() < VALVE_MAX_CONCURRENT_OPENING && find_lowest_pending(&next_idx)) {
                int next_gpio = s_valve_gpios[next_idx];
                if (next_gpio >= 0) gpio_set_level(next_gpio, 1);
                s_valve_state[next_idx] = VALVE_STATE_OPENING;
                if (s_valve_timer[next_idx] == NULL) {
                    s_valve_timer[next_idx] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)next_idx), finish_opening);
                }
                xTimerStart(s_valve_timer[next_idx], 0);
                ESP_LOGI(TAG, "Started opening pending valve %d (from close). Opening count=%d", next_idx + 1, count_opening());
                report_multistate_present_value(next_idx, 2);
            }
        }
        s_valve_state[valve_index] = VALVE_STATE_CLOSED;
        gpio_set_level(gpio, 0);
        ESP_LOGI(TAG, "Valve %d -> CLOSED (immediate). Opening count=%d", valve_index + 1, count_opening());
        /* Report physical state: 1 = Closed */
        report_multistate_present_value(valve_index, 1);
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
        gpio_set_level(gpio, 1); /* start power-hungry transition */
        s_valve_state[valve_index] = VALVE_STATE_OPENING;
        /* Report physical state: 2 = Opening */
        report_multistate_present_value(valve_index, 2);
        /* create or start timer to finish opening */
        if (s_valve_timer[valve_index] == NULL) {
            s_valve_timer[valve_index] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)valve_index), finish_opening);
        }
        xTimerStart(s_valve_timer[valve_index], 0);
        ESP_LOGI(TAG, "Started opening valve %d. Opening count=%d", valve_index + 1, count_opening());
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* Otherwise enqueue request and return success; HA sees immediate attribute accept */
    pending_enqueue(valve_index);
    ESP_LOGI(TAG, "Valve %d open request queued (concurrent limit reached).", valve_index + 1);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
