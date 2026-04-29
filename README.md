Home Assistant Coordinator — Valve Sequencer

Summary
- This repository implements a Zigbee coordinator application that exposes 11 valves
  (endpoints EP 11..21) as Home Assistant-friendly switches. Logical On/Off commands are
  accepted immediately; physical actuation is sequenced to limit the number of simultaneous
  power-hungry opens.

Key behaviour
- 11 valves (VALVE_COUNT = 11) mapped to endpoints 11..21.
- Logical control: standard ZCL On/Off cluster per endpoint. Commands return success
  immediately so Home Assistant reflects the requested state without delay.
- Physical state: each valve reports a Multistate Input cluster PresentValue with
  three states: 1=Closed, 2=Opening (or Pending), 3=Open. This allows Home Assistant to
  show real physical state separate from logical On/Off.
- Sequencing: at most VALVE_MAX_CONCURRENT_OPENING (default 4) valves may be in the
  OPENING state concurrently. Opens beyond that are recorded as VALVE_STATE_PENDING.
  When a slot frees, the driver picks the lowest-index pending valve and starts it.

Files of interest
- main/valve_driver.h — valve driver API, GPIO placeholders (VALVE_GPIO_0..VALVE_GPIO_10)
- main/valve_driver.c — sequencing logic, timers, per-valve state machine, ZCL reporting
- main/coordinator_valves.c — endpoint creation, On/Off routing and Multistate Input cluster
- test/test_valve_driver.c — Unity unit tests with deterministic timer stubs and state checks

Configuration and GPIOs
- By default the GPIO macros VALVE_GPIO_0..VALVE_GPIO_10 are set to -1 (placeholders).
  Replace these values in main/valve_driver.h with your board's GPIO numbers to enable
  hardware actuation.
- The project does not persist state across reboots — this is by design.

Testing
- Unit tests are in test/ and use Unity. They run fast and do not require hardware.
- Tests include deterministic timer stubs so you can simulate timer expiry and validate
  queue/pending behaviour and state transitions.
- To enable unit tests, build with CONFIG_UNITY enabled in your sdkconfig or use the
  provided unity-app CMake test setup.

Design Notes
- The driver models each valve as one of CLOSED, PENDING, OPENING, OPEN — this makes
  reasoning about concurrency simple and avoids separate counters that can go out of sync.
- When capacity is full, open requests set the valve to PENDING. When a slot opens the
  driver selects the lowest-index PENDING valve and starts it (not a strict FIFO queue).
- Multistate Input PresentValue is reported after state transitions and uses values
  1/2/3 as listed above. PENDING maps to the "Opening" reported value so Home Assistant
  observes activity without adding a custom numeric mapping.

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
- CLOSED + open & capacity available -> OPENING: start GPIO, set timer, report PresentValue=2 (Opening).
- CLOSED + open & capacity NOT available -> PENDING: record pending state, report PresentValue=2 (Opening/Pending).
- OPENING + timer expiry -> OPEN: set GPIO to final open, cancel timer, report PresentValue=3 (Open).
- OPENING + close request -> CLOSED: stop/delete timer, clear opening, possibly start lowest-index PENDING.
- PENDING + slot becomes available -> OPENING: chosen by lowest index, start GPIO and timer, report PresentValue=2.
- Any state + close request -> CLOSED: immediate physical close and PresentValue=1 (Closed).


Development
- Build: use ESP-IDF tooling (idf.py) as usual for your target (ESP32-H2, ESP32-C6, etc.).
- Flash: idf.py -p PORT flash monitor
- Tests: run Unity tests in the build/test runner or add CI (recommended).

Troubleshooting
- If you see unexpected counts of OPENING valves, enable unit tests under CONFIG_UNITY and
  run the tests — they include invariant checks to catch logic regressions.
- If a timer callback runs after a valve was closed, the driver ignores that callback rather
  than double-decrementing internal counters.

Contributing
- Suggest fixes or file issues in this repository. If you change GPIO mapping or
  state semantics, update tests to reflect the new behaviour.
