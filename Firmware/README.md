# Laser Parkour Firmware

## Purpose

Laser Parkour is a timed obstacle game consisting of a start button, a finish
button, and a configurable laser maze. A player starts a run by pressing the
start button, passes through the maze, and presses the finish button. Breaking
a laser beam adds a configurable time penalty. The player with the lowest
penalized time wins.

This document defines the requirements and development plan for a clean-room
rewrite of the controller and node firmware. The existing, tested hardware is
treated as fixed. The old code in `Sketches/` can be used as a behavioral
reference, but is not the basis of the new implementation.

## Terminology

- **Controller:** Raspberry Pi Pico W running the game, bus, user interface,
  and web server.
- **Node:** An ATtiny85 board connected to the sensor bus.
- **Laser node:** A node using an LDR to monitor one laser beam.
- **Start/finish node:** A node using the large button instead of an LDR.
- **Raw time:** Time between accepted start and finish events.
- **Score time:** Raw time plus all penalties.
- **Interruption:** One accepted transition from an unbroken to a broken beam.

## Game Rules

1. An operator enters or queues a player name in the web interface.
2. A run can be armed only when:
   - setup has been completed;
   - exactly one start and one finish node are present;
   - at least one laser node is present;
   - every expected node is responding; and
   - every laser beam is currently unbroken.
3. Before accepting a start, the controller resets every node event counter,
   verifies that all laser counters are zero, and records the button-counter
   baselines.
4. The player presses the start button. The controller timestamps the event and
   starts the run.
5. During the run, each accepted laser interruption increments the interruption
   count and produces audible feedback.
6. The player presses the finish button. The controller timestamps the event,
   calculates the score, stores the result, and updates the web interface.
7. The next queued player may then start when all beams are unbroken again.

The score is calculated as:

```text
score_time = raw_time + interruption_count * penalty_time
raw_time   = finish_timestamp - start_timestamp
```

The scoreboard is ordered by score time. If two score times are equal, the
lower penalty sum wins; if both are equal, the earlier completed run is listed
first.

The following values must be configurable in setup mode and persisted by the
controller:

| Setting | Unit | Initial default | Valid range |
|---|---:|---:|---:|
| Penalty per interruption | seconds | 5 | 0 to 240 |
| Maximum run time | minutes | 10 | 1 to 60 |
| Player-name length | characters | 32 | 1 to 32 |

The penalty is configurable in the web setup view. A changed value applies to
subsequent runs and does not recalculate stored scores.

An operator can abort an armed or running attempt. An aborted attempt is shown
in recent activity but is not included in the top-ten scoreboard. A run that
exceeds the maximum run time is automatically aborted.

Invalid button events are ignored and logged: finish while no run is active,
start while a run is active, and repeated button events caused by contact
bounce. Start and finish buttons must be debounced in the node firmware. A
default debounce interval of 50 ms is used and can later be adjusted at build
time.

## System States

The controller owns all game state. Nodes only own their identity,
configuration, current input state, and event counters.

| State | Description | Allowed transitions |
|---|---|---|
| `BOOT` | Initialize hardware and load persistent data | `SETUP`, `FAULT` |
| `SETUP` | Discover, commission, calibrate, and test nodes | `READY`, `FAULT` |
| `READY` | Game configured; waiting for a player | `ARMED`, `SETUP`, `FAULT` |
| `ARMED` | A current player exists; waiting for start | `RUNNING`, `READY`, `FAULT` |
| `RUNNING` | Timer and interruption counting active | `FINISHED`, `ABORTED`, `FAULT` |
| `FINISHED` | Result calculated and committed | `READY`, `ARMED` |
| `ABORTED` | Attempt ended without a score | `READY`, `ARMED` |
| `FAULT` | Required hardware is missing or inconsistent | `SETUP` |

If another player has already been queued when a run finishes, that player
becomes the current player and the state returns to `ARMED`; otherwise it
returns to `READY`. A start is accepted only in `ARMED` and only if all beams
are clear at that instant.

Entering `ARMED` includes a preparation step: the controller waits until all
beams are clear, sends `Reset counters` to every node, reads the counters back,
and enables the start event only after the reset is verified. Laser
interruptions between rounds are therefore discarded and never contribute to
the next player's score. If a laser event occurs after this reset but before
the start button is pressed, the controller disables start acceptance, waits
for all beams to be clear, and repeats the reset procedure.

At the accepted start timestamp, every laser event counter is zero. The start
button's own counter then contains the new start event; it is compared with the
baseline captured during preparation. The same baseline method identifies the
later finish event without erasing the start event.

## Fixed Hardware

The custom PCBs and level shifting have been built and tested. Custom boards
are connected through a bus, and the number of laser nodes may change whenever
the mobile game is assembled.

Pin numbers below are GPIO numbers on the Pico and port-bit names on the
ATtiny85.

### Controller

| Peripheral | Function | Pico GPIO | External component |
|---|---|---:|---|
| Speaker | Sound output | 5 | - |
| RGB LED | Red | 7 | 220 ohm |
| | Green | 8 | 220 ohm |
| | Blue | 9 | 110 ohm |
| Rotary encoder | A | 11 | Pull-up |
| | Switch | 12 | - |
| | B | 13 | Pull-up |
| LCD | SDA | 14 | - |
| | SCL | 15 | - |

### Nodes

All nodes use an ATtiny85-20U and the same firmware image. The node role and
I2C address are configuration data rather than compile-time options.

| Peripheral | ATtiny GPIO | Physical pin | External component |
|---|---|---:|---|
| LDR or button input | PB3 | 2 | Pull-up |
| Status LED | PB1 | 6 | 220 ohm |

### Sensor Bus

| Function | Pico GPIO | ATtiny pin | Notes |
|---|---:|---|---|
| SCL | 22 | PB2 | Controller is I2C master |
| SDA | 21 | PB0 | Nodes are I2C slaves |
| `FU` event | 19 | PB4 | External pull-up; button nodes only |
| `AT_RS` reset | 18 | RESET | Active-low shared hard reset |

The Pico must release `AT_RS` during normal operation and drive it low only to
reset all ATtiny nodes. `FU` is a shared active-low, open-drain signal. Start
and finish nodes assert it when their debounced button is pressed. The Pico
captures a monotonic microsecond timestamp in the GPIO interrupt handler and
then queries both button nodes over I2C to identify the source. I2C and web
processing must not be performed inside the interrupt handler.

Laser interruptions are obtained by polling and do not use `FU`. The target
bus speed is 100 kHz. If the complete physical bus is not reliable at that
speed, it may be reduced after measurement; the selected speed must be tested
with the maximum intended cable length and node count.

The supported installation contains up to 16 laser nodes plus one start and one
finish node, for 18 nodes total. The complete sensor-bus cable length is about
10 m. A strictly linear topology is not required; branches are part of the
intended installation. Bus qualification must therefore use a representative
branched 10 m harness with all 18 nodes, not only a short bench bus. The tested
branch lengths, bus speed, rise times, and error results must be recorded.
Discovery of more than 16 laser nodes is a configuration fault and prevents the
game from being armed.

## Node Commissioning and Addressing

An ATtiny85 does not provide a guaranteed unique serial number. Multiple
unconfigured nodes at the same I2C address therefore cannot be distinguished
on one bus. Nodes are commissioned one at a time:

1. Connect only the controller and the unconfigured node, or use an equivalent
   isolated programming fixture.
2. The node starts at the reserved commissioning address `0x08`.
3. In the setup page, assign a free address and select its role: laser, start,
   or finish.
4. The node stores the address, role, and configuration checksum in EEPROM and
   restarts at its assigned address.
5. Label the physical node with its assigned address.

Normal node addresses are `0x10` through `0x6F`. Addresses `0x08` through
`0x0F` are reserved for commissioning and future bootloader use. The
controller rejects duplicate addresses, multiple start nodes, multiple finish
nodes, and unknown protocol versions.

This process uses one firmware binary for every node; no source or build option
is changed per device. Recommissioning and factory reset must be supported.

## Laser Detection

Each laser node samples its ADC input continuously and maintains a filtered
value. Each node has its own persisted threshold. The setup interface displays
the raw and filtered value, current beam state, and threshold, and supports
setting one threshold or applying a value to all laser nodes.

To prevent noise near the threshold from creating events, detection uses:

- a filtered ADC value;
- separate broken and restored thresholds (hysteresis);
- a minimum stable time before accepting either state; and
- an interruption cooldown.

Initial values for development are listed below. They must be validated with
the real LDRs, lasers, ambient lighting, and intended hairstyles/clothing.

| Parameter | Initial value | Notes |
|---|---:|---|
| ADC sample period | 10 ms | 100 samples/second |
| IIR filter weight | 1/16 new, 15/16 old | Matches the old prototype behavior |
| Hysteresis | 16 ADC counts | Applied on opposite sides of the threshold |
| State stable time | 30 ms | Rejects very short changes |
| Interruption cooldown | 500 ms | Prevents one physical pass counting repeatedly |

The interruption cooldown is configurable in the web setup view, both per
laser node and with an apply-to-all action. The configured value is persisted
with the node configuration. The 500 ms value is only the initial default and
must be tuned during physical testing. The initial accepted range is 0 to
5000 ms.

One interruption is counted when the filtered beam state changes from clear to
broken and remains broken for the stable time. A continuously broken beam
counts once. It can count again only after it has returned to clear for the
stable time and the cooldown has elapsed. Different laser nodes count
independently, including simultaneous interruptions.

Each node maintains a monotonic 16-bit interruption counter. The controller
polls the counter and calculates the modulo-16-bit difference from its previous
value; reading it does not clear it. This avoids losing events between polls.
The counter resets only when the node restarts or receives an explicit reset
command. A node restart during a run is a fault and aborts that run.

### Node LED

At minimum, the LED indications are:

| Indication | Meaning |
|---|---|
| Off | No power, firmware failure, or LED disabled during a run |
| Slow blink | Uncommissioned node |
| On in setup mode | Laser node powered, responding, and beam clear |
| Fast blink in setup mode | Beam broken or threshold unsuitable |
| Short flash | Valid command or button event acknowledged |

Exact blink periods are implementation constants and must be documented in the
source. During a run, laser-node LEDs should remain off so they do not affect
the LDR reading or distract the player.

All periodic LED blinking must be generated by a hardware timer/counter output
(for example an ATtiny output-compare mode or Pico PWM hardware). Blinking must
not use periodic timer interrupts or software delay loops. Firmware may
reconfigure or enable/disable the hardware counter from its normal main-loop
state handling.

## I2C Application Protocol

The protocol is binary, versioned, and common to all node roles. Multi-byte
integers are little-endian. The controller uses command/response transactions;
nodes never initiate I2C traffic. Protocol constants and packed message layouts
must live in a shared header used by both firmware targets.

Every response includes enough status to detect a restarted, unconfigured, or
faulted node. The initial protocol must provide these operations:

| Operation | Required data |
|---|---|
| Identify | Protocol version, firmware version, role, address, boot counter |
| Read status | Input state, status flags, raw ADC, filtered ADC |
| Read events | Interruption or button-event counter |
| Configure sensor | Threshold, hysteresis, stable time, cooldown |
| Configure identity | Address and node role; commissioning mode only |
| Set operating mode | Setup or game mode |
| Save configuration | Validate and commit configuration to EEPROM |
| Reset counters | Controller preparation command before every run |
| Factory reset | Protected setup command, then restart at `0x08` |

Messages must have fixed maximum lengths suitable for the ATtiny85 RAM budget.
Configuration writes must include validation and a checksum or CRC. Unknown
commands and invalid values must leave existing configuration unchanged and set
an error status readable by the controller.

The controller polls all nodes at least 10 times per second during a run. It
uses a per-transaction timeout and retries a failed transaction twice. A node
that remains unavailable, reports a restart, or returns invalid data during a
run causes the run to abort. Outside a run it moves the controller to `FAULT`
until a successful rescan and setup check.

The concrete command IDs, byte layouts, maximum node count, and bus timing will
be defined in `Firmware/protocol.md` before implementation of the bus drivers.

## Controller Responsibilities

The Pico W firmware is responsible for:

- node discovery, commissioning, configuration, and health monitoring;
- the authoritative game state machine;
- timestamps and score calculation;
- player queue, recent results, and top-ten scoreboard;
- persistent configuration and completed results;
- Wi-Fi access point and web server;
- LCD, encoder, RGB LED, and speaker feedback; and
- fault reporting and recovery.

Timing uses the Pico's monotonic hardware timer. There is no external real-time
clock. Durations are stored and calculated as integer microseconds; formatting
and rounding occur only for display.

The operator PC supplies wall-clock time through the web interface. When the
page connects, it sends its current UTC time and local UTC offset. The
controller associates that time with its current monotonic timestamp and
derives completion timestamps for subsequent runs. Each stored result contains
an ISO 8601 completion timestamp including the offset. Client-provided time is
display metadata only and must never be used for duration or score
calculations. Until a client has supplied time after boot, completed runs use
an explicit `time unknown` value. Reconnecting may resynchronize future
timestamps but must not modify existing results.

Slow work such as flash writes, JSON generation, and HTTP handling must not run
in GPIO or timer interrupt handlers. Completed results are persisted after the
finish timestamp has been captured and the score calculated.

## Persistence and Recovery

The controller persists:

- penalty and game settings;
- Wi-Fi SSID, password, and country configuration;
- expected node addresses and roles;
- per-node laser configuration;
- top-ten scores; and
- the most recent completed/aborted attempts.

Both result collections have a fixed capacity of ten entries. No additional
history is retained. Each entry contains the player name, raw time,
interruption count, penalty used, score time, result status, and client-derived
completion timestamp. A persistent completion sequence number is used for
tie-breaking; client-provided timestamps are never trusted for ordering.

Writes must be versioned and recoverable after a partial flash write, using two
slots, a journal, or another atomic commit strategy. Flash writes should be
batched to avoid unnecessary wear. Player names must be stored and rendered as
UTF-8, validated for length, and escaped before insertion into HTML.

Nodes persist only address, role, sensor configuration, configuration version,
and checksum. They do not persist game state or interruption history.

After controller power loss, an in-progress run is considered aborted. After a
node power loss, its boot counter changes; a run in progress is aborted. The
system always returns to setup/fault checking before another run can start.

## Web Interface

The Pico W operates as a Wi-Fi access point and serves a responsive interface
for one operator PC. The AP SSID and password are persisted settings that can
be changed in the web setup view. The firmware contains documented factory
defaults so the interface remains reachable after erased or invalid storage;
deployment credentials must not be committed to the repository.

The factory Wi-Fi configuration is:

| Setting | Factory value |
|---|---|
| Country | `DE` |
| SSID | `Laser-Parkour` |
| Password | `nN7o1xt3` |

Changing the SSID or password requires explicit confirmation. The new settings
are validated and stored before the access point is restarted, which will
disconnect the current browser. Invalid settings fall back to the documented
factory defaults on the next boot. A local factory-reset action must restore
those defaults so a bad network configuration cannot permanently lock out the
operator.

### Setup View

The setup view must provide:

- bus rescan and shared node reset;
- a list of address, role, firmware/protocol version, and connection state;
- commissioning, role assignment, and factory reset;
- live raw/filtered LDR value and clear/broken state;
- individual and apply-to-all threshold controls;
- individual and apply-to-all interruption cooldown controls;
- penalty and game settings;
- current Wi-Fi SSID and password, with controls to change them;
- clear indication of every condition preventing game start; and
- a deliberate action to enter game mode.

### Game View

The game view must show:

- connection and system state;
- current player and a next-player input/queue;
- live raw time, interruption count, penalty, and score time;
- the top ten scores;
- the ten most recent attempts;
- the client-derived completion timestamp for each result;
- node or beam faults; and
- operator controls to abort the run and return to setup.

Exactly one next-player slot is provided while a run is active; there is no
larger queue. A new name may replace the queued name only after explicit
operator confirmation. Duplicate player names are allowed. Blank names are
rejected.

The browser receives state updates without reloading the page. Server-Sent
Events are preferred for controller-to-browser updates because the primary data
flow is one-way; ordinary HTTP requests are used for commands. Commands must be
validated against the controller state and return structured success or error
responses. A disconnected browser does not stop an active run.

## Local User Interface

The RGB LED, LCD, encoder, and speaker provide basic operation without relying
on the browser for immediate feedback. The LCD provides detailed information,
the speaker provides immediate game-event feedback, and the RGB LED is reserved
for system health, readiness, and faults. Detailed game state is shown in the
web interface and is not encoded as an RGB color.

### LCD and Encoder

The LCD uses a page-based interface:

- At the top level, rotating left or right switches between pages.
- Clicking enters the selected page.
- Inside a page, rotating scrolls through its fields and available actions.
- Clicking a field selects it or executes its action.
- Editable values use rotate to change and click to confirm.
- Every nested page contains a `Back` action.
- Destructive or disruptive actions require a separate confirmation step.

There is no required wraparound between the first and last page or option.
Encoder input is debounced, and one physical detent produces one navigation
step. Navigation must remain responsive while the game, bus polling, and web
server are active.

The minimum top-level pages are:

| Page | Minimum information or actions |
|---|---|
| Status | Controller state, current player, live/result time, interruptions |
| Sensors | Node counts, beam-ready state, and active fault |
| Network | SSID, password, and web-interface IP address |
| System | Firmware version, rescan, return to setup, and reset actions |

Long values may scroll or be split across multiple views. The exact wording and
field order must be evaluated on the physical LCD.

### Sounds

The speaker provides three distinct non-blocking sound patterns:

| Event | Sound character |
|---|---|
| Start button accepted | Short, positive start confirmation |
| Laser interruption accepted | Short warning tone, clearly unlike start |
| Finish button accepted | Longer success pattern, clearly unlike both others |

Sounds are generated with Pico PWM hardware. Playback must not block timestamp
capture, bus polling, the game state machine, or the web server. Initial
frequencies, durations, pauses, and repetition counts will be selected during
implementation and finalized by listening tests on the actual speaker.

### Controller RGB LED

The controller RGB LED is a system-health and fault indicator. Higher-priority
conditions override lower-priority indications.

| Priority | Condition | Initial indication |
|---:|---|---|
| 1 | Internal, storage, or unrecoverable controller fault | Fast red blink |
| 2 | Required node missing, sensor-bus fault, or invalid node configuration | Slow red blink |
| 3 | Wi-Fi access point failed or stopped unexpectedly | Magenta blink |
| 4 | Setup incomplete or one or more beams currently broken | Amber with blue heartbeat |
| 5 | System healthy and ready for operation | Blue heartbeat |

During boot and AP startup, the LED blinks blue. Once the access point is
active, has a valid IP address, and the HTTP server is ready to accept requests,
a brief blue heartbeat is shown. The heartbeat is also visible over the amber
setup/readiness warning. When no warning is active, the LED remains off between
blue heartbeat pulses. These indications remain the same throughout all game
states. Loss of the AP or web server removes the heartbeat and activates the
higher-priority AP-fault indication.

The LED controller receives controller health, storage health, bus/node health,
AP/web health, and setup readiness as separate inputs and selects the
highest-priority active indication in one place. LED patterns must not be
scattered across unrelated modules. Periodic blinking and the heartbeat follow
the hardware-counter requirement defined for LEDs. Exact brightness, blink
rates, and heartbeat timing are finalized on the physical hardware.

## Implementation Language and Build

PlatformIO is the supported development, build, upload, and serial-monitor
workflow in Visual Studio Code. The current project setup is:

- Pico W controller: C++ using the Arduino-Pico core. This provides the Pico W
  Wi-Fi support used by the controller smoke test.
- ATtiny85: C11 with the PlatformIO Atmel AVR platform, AVR-GCC, and AVR Libc.
- Portable tests: a PlatformIO native environment where possible.

The controller and node are independent PlatformIO projects in
`Firmware/controller` and `Firmware/node`. Environment-specific upload
protocols, programmer ports, and local SDK paths should be overridable without
editing committed files. The VS Code PlatformIO extension invokes the same
environments as the CLI commands below.

Third-party libraries should be kept minimal. Compiler warnings are enabled and
treated as errors in continuous integration where practical. Hardware-specific
drivers are separated from game and protocol logic so that state-machine,
scoring, persistence encoding, and protocol tests can run on a desktop host.

Suggested source layout:

```text
Firmware/
  protocol.md
  shared/              Protocol types and portable utilities
  controller/
    platformio.ini
    src/               Production entry point
    smoke/             Preserved combined hardware smoke test
    drivers/           Pico hardware access
    game/              State machine, scoring, and player queue
    bus/               Discovery and node communication
    storage/           Versioned persistent data
    web/               HTTP/SSE server and static assets
    ui/                LCD, encoder, LED, and sound
  node/
    platformio.ini
    src/               Production entry point
    smoke/             Preserved combined hardware smoke test
    drivers/           ADC, EEPROM, GPIO, timer, USI/I2C
    app/               Role behavior and event detection
  tests/               Host-side unit and protocol tests
```

### Hardware smoke tests

Each board has one preserved, combined hardware smoke test. These programs are
kept separate from production firmware so they remain available for assembly
verification and later fault diagnosis. The normal production environments are
the defaults; their `src/main.*` files are currently minimal placeholders.

From `Firmware/controller`:

```sh
# Build the production entry point (default environment).
pio run

# Build or upload the complete controller smoke test.
pio run -e smoke-controller
pio run -e smoke-controller -t upload
```

From `Firmware/node`:

```sh
# Build the production entry point (default environment).
pio run

# Build or upload the complete node smoke test through the Arduino Uno ISP.
pio run -e smoke-node
pio run -e smoke-node -t upload
```

In VS Code, select `smoke-controller` or `smoke-node` in the PlatformIO project
environment selector before choosing Build or Upload. Select `rpipicow` or
`attiny85` to return to production firmware.

The detailed behavior and pass/fail criteria are maintained beside the tests:

- `controller/smoke/README.md` for the access point, serial output, RGB LED,
  speaker, LCD, and encoder;
- `node/smoke/README.md` for the hardware-timed status LED and LDR input.

## Development Roadmap

Each milestone should leave a testable system. Later work must not begin by
depending on an undefined protocol or game rule.

### 0. Specification and Toolchain

- Resolve the open decisions at the end of this document.
- Record maximum tested node count and cable length.
- Create PlatformIO controller, node, and native-test environments with
  warning-clean minimal firmware.
- Document building, uploading, serial monitoring, and running tests from both
  the PlatformIO CLI and VS Code.
- Add code formatting configuration.

**Exit criterion:** both targets build reproducibly through PlatformIO, flash
successfully, and run a basic LED/speaker smoke test. Host tests run through the
PlatformIO native environment.

### 1. Protocol and ATtiny Platform

- Write `protocol.md` with exact messages and shared C definitions.
- Implement ATtiny timers, ADC, EEPROM configuration, status LED, reset
  detection, and USI-based I2C slave transport.
- Verify LED blink patterns on the hardware timer output without timer
  interrupts.
- Implement commissioning and EEPROM validation/factory reset.

**Exit criterion:** one firmware image can commission each node role, retain it
across power cycles, and reliably answer repeated protocol tests.

### 2. Pico Bus Manager

- Implement bus scan, node inventory, protocol/version validation, retries, and
  fault handling.
- Test shared reset and `FU` timestamp capture.
- Measure bus reliability at the intended cable length and node count.

**Exit criterion:** the controller discovers all test nodes, distinguishes the
two button roles, timestamps their events, and detects disconnects/restarts.

### 3. Laser Detection

- Implement filtering, hysteresis, stable-time validation, cooldown, and event
  counters on laser nodes.
- Build a controller diagnostic view over serial before depending on the web UI.
- Tune defaults using actual lasers and representative ambient conditions.

**Exit criterion:** deliberate beam breaks count correctly and noise, slow
threshold crossings, and ponytail-like repeated movement pass recorded tests.

### 4. Game Engine

- Implement the controller state machine, player queue, timestamp handling,
  between-round counter reset, scoring, aborts, timeouts, and fault transitions.
- Add host unit tests for every state transition and scoring edge case.
- Add speaker game-event feedback and RGB system-health indication.

**Exit criterion:** complete games run through serial/local controls with
deterministic scores and correct behavior for invalid and fault events.

### 5. Persistence

- Implement versioned, atomic controller storage and migration/default handling.
- Store settings, node inventory/configuration, recent attempts, and top scores.
- Test interrupted writes, corrupted records, and both result-list capacity
  limits.

**Exit criterion:** completed data survives reboot and simulated incomplete or
corrupt writes recover without preventing setup.

### 6. Web Interface

- Implement AP configuration, editable persisted credentials, HTTP API, SSE
  state stream, setup view, and game view.
- Implement browser time synchronization and unknown-time behavior.
- Validate and escape all user input.
- Test browser disconnect/reconnect during setup and during a run.

**Exit criterion:** one PC can configure the system and operate a full game,
including queuing the next player, without serial access.

### 7. Local UI and Integration

- Complete LCD/encoder navigation, including SSID, password, and IP display,
  and document all LED and sound patterns.
- Verify RGB priority handling by injecting controller, storage, bus, node, AP,
  web-server, and setup-readiness conditions.
- Run full setup, game, abort, timeout, disconnect, reset, and power-loss tests.
- Measure timing accuracy and worst-case response under simultaneous web and bus
  load.

**Exit criterion:** the assembled game meets all requirements in this document
for repeated multi-player sessions.

### 8. Release and Maintenance

- Document flashing, commissioning, setup, backup/reset, and troubleshooting.
- Record firmware versions and build artifacts.
- Tag the first usable release and maintain a change log.

**Exit criterion:** the system can be rebuilt, installed, commissioned, and
operated using only repository documentation.

## Open Decisions

These decisions do not block the initial toolchain work, but must be resolved
before their corresponding milestone:

- Exact LCD wording and field ordering after physical evaluation.
- Final RGB brightness, blink rates, and heartbeat timing after physical
  evaluation.
- Final sound frequencies and timing after listening tests.
