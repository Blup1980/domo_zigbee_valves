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

typedef enum { VALVE_STATE_CLOSED = 0, VALVE_STATE_OPENING, VALVE_STATE_OPEN } valve_state_t;

/* runtime state */
static valve_state_t s_valve_state[VALVE_COUNT];
static TimerHandle_t  s_valve_timer[VALVE_COUNT]; /* timer used to finish opening after 3 minutes */
static SemaphoreHandle_t s_lock;
static int s_opening_count = 0; /* number of valves currently in opening transition */

/* queue of pending valve indices that want to open when slot is available */
static uint8_t s_pending_queue[VALVE_COUNT];
static int s_pending_head = 0;
static int s_pending_tail = 0;

/* constants */
#define VALVE_OPENING_MS (3 * 60 * 1000) /* 3 minutes */
#define VALVE_MAX_CONCURRENT_OPENING 4

static void pending_enqueue(uint8_t idx)
{
    /* avoid duplicate entries */
    for (int i = s_pending_head; i != s_pending_tail; i = (i + 1) % VALVE_COUNT) {
        if (s_pending_queue[i] == idx) return;
    }
    int next = (s_pending_tail + 1) % VALVE_COUNT;
    if (next == s_pending_head) {
        ESP_LOGW(TAG, "Pending queue full, dropping request for valve %d", idx + 1);
        return;
    }
    s_pending_queue[s_pending_tail] = idx;
    s_pending_tail = next;
}

#ifdef CONFIG_UNITY
valve_drv_state_t valve_driver_test_get_state(uint8_t valve_index)
{
    if (valve_index >= VALVE_COUNT) return VALVE_DRV_STATE_CLOSED;
    return (valve_drv_state_t)s_valve_state[valve_index];
}

int valve_driver_test_get_opening_count(void)
{
    return s_opening_count;
}

int valve_driver_test_get_pending_length(void)
{
    if (s_pending_tail >= s_pending_head) return s_pending_tail - s_pending_head;
    return VALVE_COUNT - (s_pending_head - s_pending_tail);
}

esp_err_t valve_driver_test_finish_open(uint8_t valve_index)
{
    if (valve_index >= VALVE_COUNT) return ESP_ERR_INVALID_ARG;
    /* call the finish_opening logic directly to simulate timer expiry */
    if (s_valve_timer[valve_index] != NULL) {
        finish_opening(s_valve_timer[valve_index]);
        return ESP_OK;
    }
    /* if no timer, still set state to open and report */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_valve_state[valve_index] = VALVE_STATE_OPEN;
    if (s_opening_count > 0) s_opening_count--;
    xSemaphoreGive(s_lock);
    report_multistate_present_value(valve_index, 3);
    return ESP_OK;
}
#endif

static bool pending_dequeue(uint8_t *out_idx)
{
    if (s_pending_head == s_pending_tail) return false;
    *out_idx = s_pending_queue[s_pending_head];
    s_pending_head = (s_pending_head + 1) % VALVE_COUNT;
    return true;
}

static void pending_remove(uint8_t idx)
{
    int read = s_pending_head;
    int write = s_pending_head;
    while (read != s_pending_tail) {
        uint8_t v = s_pending_queue[read];
        if (v != idx) {
            s_pending_queue[write] = v;
            write = (write + 1) % VALVE_COUNT;
        }
        read = (read + 1) % VALVE_COUNT;
    }
    s_pending_tail = write;
}

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

static void finish_opening(TimerHandle_t timer)
{
    uint32_t idx = (uint32_t)pvTimerGetTimerID(timer);
    if (idx >= VALVE_COUNT) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int gpio = s_valve_gpios[idx];
    if (gpio >= 0) {
        gpio_set_level(gpio, 1); /* ensure open */
    }
    s_valve_state[idx] = VALVE_STATE_OPEN;
    s_opening_count--;
    ESP_LOGI(TAG, "Valve %d finished opening (now OPEN). Opening count=%d", idx + 1, s_opening_count);

    /* start next pending if any */
    uint8_t next_idx;
    if (s_opening_count < VALVE_MAX_CONCURRENT_OPENING && pending_dequeue(&next_idx)) {
        /* begin opening next */
        int next_gpio = s_valve_gpios[next_idx];
        if (next_gpio >= 0) gpio_set_level(next_gpio, 1); /* start power-hungry transition */
        s_valve_state[next_idx] = VALVE_STATE_OPENING;
        s_opening_count++;
        /* start/arm timer for next_idx */
        if (s_valve_timer[next_idx] == NULL) {
            s_valve_timer[next_idx] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)next_idx), finish_opening);
        }
        xTimerStart(s_valve_timer[next_idx], 0);
        ESP_LOGI(TAG, "Dequeued and started opening valve %d. Opening count=%d", next_idx + 1, s_opening_count);
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
    s_pending_head = s_pending_tail = 0;
    s_opening_count = 0;

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
        /* remove from pending queue if queued */
        pending_remove(valve_index);
        if (s_valve_timer[valve_index] != NULL) {
            xTimerStop(s_valve_timer[valve_index], 0);
            xTimerDelete(s_valve_timer[valve_index], 0);
            s_valve_timer[valve_index] = NULL;
        }
        /* if it was opening, decrement count */
        if (s_valve_state[valve_index] == VALVE_STATE_OPENING) {
            if (s_opening_count > 0) s_opening_count--;
        }
        s_valve_state[valve_index] = VALVE_STATE_CLOSED;
        gpio_set_level(gpio, 0);
        ESP_LOGI(TAG, "Valve %d -> CLOSED (immediate). Opening count=%d", valve_index + 1, s_opening_count);
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

    /* If we have capacity, start opening immediately */
    if (s_opening_count < VALVE_MAX_CONCURRENT_OPENING) {
        gpio_set_level(gpio, 1); /* start power-hungry transition */
        s_valve_state[valve_index] = VALVE_STATE_OPENING;
        s_opening_count++;
        /* Report physical state: 2 = Opening */
        report_multistate_present_value(valve_index, 2);
        /* create or start timer to finish opening */
        if (s_valve_timer[valve_index] == NULL) {
            s_valve_timer[valve_index] = xTimerCreate("valve_timer", pdMS_TO_TICKS(VALVE_OPENING_MS), pdFALSE, (void *)((uintptr_t)valve_index), finish_opening);
        }
        xTimerStart(s_valve_timer[valve_index], 0);
        ESP_LOGI(TAG, "Started opening valve %d. Opening count=%d", valve_index + 1, s_opening_count);
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* Otherwise enqueue request and return success; HA sees immediate attribute accept */
    pending_enqueue(valve_index);
    ESP_LOGI(TAG, "Valve %d open request queued (concurrent limit reached).", valve_index + 1);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool valve_driver_get_physical_state(uint8_t valve_index)
{
    if (valve_index >= VALVE_COUNT) return false;
    int gpio = s_valve_gpios[valve_index];
    if (gpio < 0) return false;
    int level = gpio_get_level(gpio);
    return level != 0;
}
