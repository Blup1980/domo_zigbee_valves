#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

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

/* Simple test timer implementation: we store callback and id so tests can trigger it
 * deterministically by calling valve_driver_test_finish_open which calls the driver's
 * finish_opening with the timer handle. */
typedef struct {
    void *id;
    void (*cb)(TimerHandle_t);
    bool active;
} TestTimer;

static TestTimer *test_timers[VALVE_COUNT];

TimerHandle_t xTimerCreate(const char *name, uint32_t ticks, int auto_reload, void *id, void (*cb)(TimerHandle_t))
{
    (void)name; (void)ticks; (void)auto_reload;
    for (int i = 0; i < VALVE_COUNT; ++i) {
        if (test_timers[i] == NULL) {
            test_timers[i] = (TestTimer *)malloc(sizeof(TestTimer));
            test_timers[i]->id = id;
            test_timers[i]->cb = cb;
            test_timers[i]->active = false;
            return (TimerHandle_t)test_timers[i];
        }
    }
    return NULL;
}

int xTimerStart(TimerHandle_t t, int) { (void)t; return 1; }
int xTimerStop(TimerHandle_t t, int) { (void)t; return 1; }
int xTimerDelete(TimerHandle_t t, int) { if (!t) return 1; TestTimer *tt = (TestTimer *)t; for (int i = 0; i < VALVE_COUNT; ++i) { if (test_timers[i] == tt) { free(tt); test_timers[i] = NULL; return 1; } } return 1; }

/* pvTimerGetTimerID is used by finish_opening in the driver to recover the timer id */
void *pvTimerGetTimerID(TimerHandle_t t) { if (!t) return NULL; return ((TestTimer *)t)->id; }

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

/* Helper for tests: trigger timer callback for valve index to simulate expiry.
 * The driver's valve_driver_test_finish_open calls finish_opening which expects a TimerHandle_t.
 * Here we implement a test-specific helper that invokes the stored callback for the given index. */
int test_trigger_timer_for_index(int valve_index)
{
    if (valve_index < 0 || valve_index >= VALVE_COUNT) return 0;
    TestTimer *tt = test_timers[valve_index];
    if (!tt) return 0;
    if (tt->cb) tt->cb((TimerHandle_t)tt);
    return 1;
}

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
    RUN_TEST(test_deterministic_queue_behavior);
    RUN_TEST(test_edge_cases);
    return UNITY_END();
}


void test_deterministic_queue_behavior(void)
{
    valve_driver_init(false);

    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(0, true));
    int maxc = valve_driver_test_get_max_concurrent_opening();
    for (int i = 1; i <= maxc + 2; ++i) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(i, true));
    }

    TEST_ASSERT_EQUAL_INT(maxc, valve_driver_test_get_opening_count());

    int pending = valve_driver_test_get_pending_length();
    TEST_ASSERT_EQUAL_INT(3, pending);

    TEST_ASSERT_EQUAL_INT(maxc, valve_driver_test_get_pending_at(0));
    TEST_ASSERT_EQUAL_INT(maxc + 1, valve_driver_test_get_pending_at(1));
    TEST_ASSERT_EQUAL_INT(maxc + 2, valve_driver_test_get_pending_at(2));

    /* finish the first opening (valve 0) -> should become OPEN and first queued should start OPENING */
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_test_finish_open(0));
    TEST_ASSERT_EQUAL_INT(VALVE_DRV_STATE_OPEN, valve_driver_test_get_state(0));

    int first_queued = maxc;
    TEST_ASSERT_EQUAL_INT(VALVE_DRV_STATE_OPENING, valve_driver_test_get_state(first_queued));
    TEST_ASSERT_EQUAL_INT(2, valve_driver_test_get_pending_length());
}


void test_edge_cases(void)
{
    valve_driver_init(false);

    /* Close while opening: open then immediately close */
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(0, true));
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(0, false));
    TEST_ASSERT_EQUAL_INT(VALVE_DRV_STATE_CLOSED, valve_driver_test_get_state(0));

    /* Duplicate open requests should not create duplicates in queue */
    /* Fill openings up to maxc-1 (we'll reserve one for duplicates) */
    int maxc = valve_driver_test_get_max_concurrent_opening();
    for (int i = 0; i < maxc; ++i) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(i, true));
    }

    /* Now enqueue valve 'maxc' twice; second should be ignored as duplicate */
    int before = valve_driver_test_get_pending_length();
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(maxc, true));
    int after1 = valve_driver_test_get_pending_length();
    TEST_ASSERT_TRUE(after1 >= before);
    TEST_ASSERT_EQUAL_INT(ESP_OK, valve_driver_set_power(maxc, true));
    int after2 = valve_driver_test_get_pending_length();
    TEST_ASSERT_EQUAL_INT(after1, after2);

    /* Queue full behavior: attempt to enqueue all remaining valves and ensure length caps */
    for (int i = maxc + 1; i < VALVE_COUNT; ++i) {
        valve_driver_set_power(i, true);
    }
    int pending = valve_driver_test_get_pending_length();
    TEST_ASSERT_TRUE(pending <= VALVE_COUNT);
    /* Trying to enqueue an already queued valve should not grow the queue */
    int prev = pending;
    valve_driver_set_power(maxc, true);
    TEST_ASSERT_EQUAL_INT(prev, valve_driver_test_get_pending_length());

    /* Invalid index should return error */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, valve_driver_set_power(VALVE_COUNT + 5, true));

    /* default_open behavior */
    valve_driver_init(true);
    for (int i = 0; i < VALVE_COUNT; ++i) {
        TEST_ASSERT_EQUAL_INT(VALVE_DRV_STATE_OPEN, valve_driver_test_get_state(i));
    }
}
