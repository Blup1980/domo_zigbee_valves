Home Assistant Zigbee Router — Valve Sequencer

Summary
- This repository implements an ESP-IDF Zigbee *router* application that exposes 11 valves
  (endpoints EP 10..20) as Home Assistant-friendly On/Off entities. On/Off writes are accepted
  immediately; physical actuation is sequenced to limit the number of simultaneous
  power-hungry opens.

Key behaviour
- 11 valves (`VALVE_COUNT = 11`) mapped to endpoints `10..20` (`ESP_ZIGBEE_HA_FIRST_EP_ID=10`).
- Each endpoint is created as a ZHA "mains power outlet" device via `ezb_zha_create_mains_power_outlet()`.
- Logical control: ZCL SetAttributeValue handling for the On/Off cluster (`EZB_ZCL_CLUSTER_ID_ON_OFF`)
  and the On/Off attribute (`EZB_ZCL_ATTR_ON_OFF_ON_OFF_ID`). The handler maps
  `endpoint -> valve_index` with `valve_index = ep - ESP_ZIGBEE_HA_FIRST_EP_ID` and calls
  `valve_driver_set_power(valve_index, on)`.
- Physical actuation: GPIO outputs (active-high) with a per-valve state machine:
  `CLOSED`, `PENDING`, `OPENING`, `OPEN`.
- Sequencing: at most `VALVE_MAX_CONCURRENT_OPENING` (currently hardcoded to `4` in
  `main/valve_driver.c`) may be in `OPENING` concurrently.
  Additional open requests move the valve to `PENDING`. When a slot frees, the driver starts the
  lowest-index `PENDING` valve (not strict FIFO).
- Timing: a valve remains `OPENING` for `VALVE_OPENING_MS` (currently `2 minutes`) and then
  transitions to `OPEN` via a FreeRTOS software timer.

State indication
- The current firmware does *not* create/report a Multistate Input (or any other) cluster for the
  physical/sequencing state. Home Assistant primarily sees the logical On/Off state of each endpoint.
- An LED strip is used as a local indicator via `valve_changed_callback()` (`main/router_valves.c`),
  which sets LEDs based on the *driver state*:
  `CLOSED=red`, `OPENING=yellow`, `OPEN=green`, `PENDING=blue`.

Files of interest
- main/router_valves.c — Zigbee startup/commissioning, endpoint creation, On/Off routing, LED indication
- main/router_valves.h — Zigbee/router configuration constants (channels, endpoints, storage)
- main/valve_driver.h — valve driver API and GPIO mapping (`VALVE_GPIO_0..VALVE_GPIO_10`)
- main/valve_driver.c — sequencing logic, timers, per-valve state machine
- main/light_driver.* — LED strip helper used by `valve_changed_callback()`
- main/alarm_timer.* — simple `esp_timer` wrapper used for commissioning retries

Configuration and GPIOs
- GPIO mapping is defined in `main/valve_driver.h` (`VALVE_GPIO_0..VALVE_GPIO_10`).
  Edit these values to match your board wiring.
- LED strip GPIO is `CONFIG_STRIP_LED_GPIO` in `main/light_driver.h`.
- Zigbee storage uses an NVS partition named `zb_storage` (`ESP_ZIGBEE_STORAGE_PARTITION_NAME`).
- The project does not persist valve state across reboots.

Testing
- This repository previously carried Unity unit tests; check the git history if you want to
  re-enable/restore them.

Design Notes
- The driver models each valve as one of `CLOSED`, `PENDING`, `OPENING`, `OPEN` — this makes
  reasoning about concurrency simple and avoids separate counters that can go out of sync.
- When capacity is full, open requests set the valve to PENDING. When a slot opens the
  driver selects the lowest-index PENDING valve and starts it (not a strict FIFO queue).
- `PENDING` is only assigned when an *open request* arrives while capacity is full and the valve is
  currently `CLOSED` (duplicate pending requests are ignored).

State Machine (ASCII diagram)

  Open Request                         Timer Expiry
  -------------                        -------------
  [CLOSED] -------(open, capacity)----> [OPENING] ----(timer expires)----> [OPEN]
     |                                   |  ^
     |                                   |  | Close Request
     |                                   |  |
     |                                   v  |
     +--(open, no capacity)--> [PENDING] -----(close or cancel)----> [CLOSED]

Notes on transitions
- CLOSED + open & capacity available -> OPENING: start GPIO, set timer, report using yellow LED.
- CLOSED + open & capacity NOT available -> PENDING: record pending state, report using blue LED.
- OPENING + timer expiry -> OPEN: set GPIO to final open, cancel timer, report using green LED.
- OPENING + close request -> CLOSED: stop/delete timer, set GPIO low, and if a `PENDING` valve exists
  it is immediately started (`PENDING -> OPENING`). LED indication: closed valve becomes red; any started
  pending valve becomes yellow.
- PENDING + slot becomes available -> OPENING: chosen by lowest index, start GPIO and timer, report using yellow LED.
- Any state + close request -> CLOSED: immediate physical close and report using red LED.


Development
- Build: use ESP-IDF tooling (idf.py) as usual for your target (ESP32-H2, ESP32-C6, etc.).
- Flash: idf.py -p PORT flash monitor
- Zigbee role: configured as a router in `main/router_valves.h` (`EZB_NWK_DEVICE_TYPE_ROUTER`).

Commissioning/runtime flow (as implemented)
- `app_main()` initializes NVS and the Zigbee NVS partition (`zb_storage`), then starts a Zigbee task.
- Zigbee task calls `esp_zigbee_init()`, registers the commissioning signal handler, creates endpoints,
  starts the stack, then enters `esp_zigbee_launch_mainloop()`.
- Valve GPIO + LED strip initialization is deferred until Zigbee startup succeeds (see `deferred_driver_init()`).
- On first start (factory-new), the device runs network steering; otherwise it just logs reboot.

Troubleshooting
- If a timer callback runs after a valve was closed, the driver ignores that callback rather
  than transitioning the valve twice.

