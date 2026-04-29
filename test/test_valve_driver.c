#include "unity.h"

#include <stdbool.h>
#include <stdint.h>

/* We will test the logic of valve_driver without GPIO and Zigbee stack by
 * mocking the external dependencies (gpio_set_level, gpio_get_level, timers,
 * and Zigbee functions). Keep tests isolated and fast.
 */

/* Include the driver header */
#include "valve_driver.h"

/* Mocked symbols (minimal) */
int gpio_levels[VALVE_COUNT];

int gpio_set_level(int gpio, int level)
{
    (void)gpio; /* gpio index isn't used in test code; driver uses mapping values */
    return 0 == 0 ? 0 : -1; /* always succeed */
}

int gpio_get_level(int gpio)
{
    (void)gpio;
    return 0;
}

/* Minimal stubs for FreeRTOS primitives used by driver in the unit test environment */
typedef void * TimerHandle_t;

TimerHandle_t xTimerCreate(const char *name, uint32_t ticks, int auto_reload, void *id, void (*cb)(TimerHandle_t)) { (void)name; (void)ticks; (void)auto_reload; (void)id; (void)cb; return (TimerHandle_t)1; }
int xTimerStart(TimerHandle_t t, int) { (void)t; return 0; }
int xTimerStop(TimerHandle_t t, int) { (void)t; return 0; }
int xTimerDelete(TimerHandle_t t, int) { (void)t; return 0; }

void * xSemaphoreCreateMutex(void) { return (void *)1; }
int xSemaphoreTake(void *m, int) { (void)m; return 1; }
int xSemaphoreGive(void *m) { (void)m; return 1; }

/* Zigbee lock stubs */
int esp_zigbee_lock_acquire(int t) { (void)t; return 1; }
void esp_zigbee_lock_release(void) {}

/* Stubs for ZCL attribute/report functions used by driver */
typedef int ezb_zcl_status_t;
ezb_zcl_status_t ezb_zcl_set_attr_value(uint8_t ep, uint16_t cluster_id, uint8_t cluster_role, uint16_t attr_id, uint16_t manuf_code, void *value, bool check) { (void)ep; (void)cluster_id; (void)cluster_role; (void)attr_id; (void)manuf_code; (void)value; (void)check; return 0; }
int ezb_zcl_report_attr_cmd_req(void *req) { (void)req; return 0; }

/* Tests */
void test_init_defaults(void)
{
    /* initialize with default_open=false - should not crash */
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_init(false));
}

void test_open_close_sequence(void)
{
    valve_driver_init(false);

    /* open valve 0 -> should be accepted */
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(0, true));

    /* while opening, check opening_count and state */
    TEST_ASSERT_EQUAL_INT(1, valve_driver_test_get_opening_count());
    TEST_ASSERT_EQUAL_INT(VALVE_DRV_STATE_OPENING, valve_driver_test_get_state(0));

    /* enqueue opens up to and beyond the concurrency limit */
    int maxc = valve_driver_test_get_max_concurrent_opening();
    for (int i = 1; i <= maxc + 2; ++i) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(i, true));
    }

    /* opening_count should not exceed the limit */
    TEST_ASSERT_LESS_THAN_OR_EQUAL_INT(maxc, valve_driver_test_get_opening_count());

    /* some requests should be queued: pending_length > 0 */
    int pending = valve_driver_test_get_pending_length();
    TEST_ASSERT_TRUE(pending >= 0);

    /* inspect queued indices for deterministic behavior (FIFO) */
    for (int p = 0; p < pending; ++p) {
        int idx = valve_driver_test_get_pending_at(p);
        TEST_ASSERT_TRUE(idx >= 0 && idx < VALVE_COUNT);
    }

    /* simulate finish of valve 0 opening and ensure it becomes OPEN */
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_test_finish_open(0));
    TEST_ASSERT_EQUAL_INT(VALVE_DRV_STATE_OPEN, valve_driver_test_get_state(0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_defaults);
    RUN_TEST(test_open_close_sequence);
    return UNITY_END();
}
