# Rowing Machine Controller

Wireless linear rowing-machine / reciprocating carriage controller built around a Seeed Studio XIAO ESP32-S3, TB6600 stepper driver, and battery-powered ESP32-C3 wireless limit switches.

## Project goals

- Calibrate the usable linear travel automatically from end-stop switches.
- Run a repeatable automatic sweep between endpoints.
- Retain a manual carriage-position mode.
- Provide simple browser-based control over Wi-Fi.
- Eliminate long HOME/END wiring by using ESP-NOW wireless limit nodes.
- Keep the wireless limit nodes battery powered and low maintenance.

## System architecture

```text
HOME microswitch -> XIAO ESP32-C3, node 1 --\
                                             \ ESP-NOW
END microswitch  -> XIAO ESP32-C3, node 2 ----> XIAO ESP32-S3
                                                    |
Phone / PC <------------ Wi-Fi web UI --------------+
                                                    |
Mode toggle + 10k potentiometer --------------------+
                                                    |
                                               STEP / DIR
                                                    |
                                                 TB6600
                                                    |
                                              Stepper motor
                                                    |
                                           Linear rail carriage
```

## Main components

| Component | Model / detail | Function |
|---|---|---|
| Main controller | Seeed Studio XIAO ESP32-S3 | Motion control, ESP-NOW receiver, Wi-Fi web server |
| Wireless limit nodes | Seeed Studio XIAO ESP32-C3 | Battery-powered HOME and END transmitters |
| Stepper driver | TB6600 | STEP/DIR motor driver |
| Motor | NEMA 17 stepper | Carriage actuation |
| Motion system | Linear rail/carriage | Reciprocating travel |
| Mode selector | ON-OFF-ON toggle | AUTO / IDLE / MANUAL |
| Manual input | 10 kOhm potentiometer | Manual target position |
| Limit switches | Mechanical microswitches | HOME and END sensing |
| Main supply | 24 V DC supply | Motor/driver power |
| Logic converter | LM2596 buck converter | 24 V to 5 V logic supply |
| Wireless-node supply | 1S Li-ion battery | Portable C3 power |
| Battery considered | Samsung INR18650-29E, 3.6 V, 2900 mAh | Long runtime for wireless limit nodes |

## Main-controller pin assignment

| XIAO ESP32-S3 GPIO | Connection |
|---|---|
| GPIO1 | 10 kOhm potentiometer |
| GPIO2 | AUTO selector input |
| GPIO3 | MANUAL selector input |
| GPIO4 | TB6600 STEP / PUL |
| GPIO5 | TB6600 DIR |

The original wired limit inputs on GPIO43 and GPIO8 were removed from the active design after migration to ESP-NOW limits.

## TB6600 wiring

```text
GPIO4 -> PUL+
GPIO5 -> DIR+

PUL- -> GND
DIR- -> GND
ENA- -> GND
```

The S3 and TB6600 logic ground must be common.

## Motion parameters

Current working values:

```cpp
const uint32_t HOME_SPEED      = 1500;
const uint32_t MAX_SPEED       = 17000;
const uint32_t ACCEL           = 17000;

const uint32_t FAST_HOME_SPEED = 1800;
const uint32_t SLOW_HOME_SPEED = 200;

const int FAST_BACKOFF         = 100;
const int SLOW_SEARCH          = 1000;

const int SEARCH_STEP          = 500;
const int BACKOFF              = 100;
const int POSITION_DEADBAND    = 20;
const int MANUAL_DEADBAND      = 50;
```

FastAccelStepper 1.2.7 is used for non-blocking motion control.

## Calibration state machine

The controller measures its own usable travel at startup.

```text
FIND_HOME
  -> BACKOFF_HOME
  -> FIND_HOME_SLOW
  -> set current position = 0
  -> FIND_END
  -> BACKOFF_END
  -> FIND_END_SLOW
  -> record endPos
  -> READY
```

The slow second pass at each endpoint improves repeatability. Once calibration completes, the controller performs one center move to `endPos / 2`.

A previous bug repeatedly issued a center command while in READY, which fought the manual-position logic and made the carriage wander. READY was changed to perform the center move only once.

## AUTO mode

AUTO sweeps continuously between `0` and `endPos`.

The original potentiometer-based speed control was replaced with five fixed browser-selectable levels because discrete levels were easier to understand and reproduce.

| Level | Speed |
|---|---:|
| 1 | 3000 steps/s |
| 2 | 6000 steps/s |
| 3 | 9000 steps/s |
| 4 | 13000 steps/s |
| 5 | 17000 steps/s |

```cpp
uint32_t getAutoSpeed() {
  switch (wifiSpeedLevel) {
    case 1: return 3000;
    case 2: return 6000;
    case 3: return 9000;
    case 4: return 13000;
    case 5: return 17000;
    default: return 9000;
  }
}
```

## MANUAL mode

The potentiometer is mapped to a carriage target position from HOME to END.

```cpp
filteredManualPot = filteredManualPot * 0.90f
                  + analogRead(POT_PIN) * 0.10f;

int32_t target = map(
  (int)filteredManualPot,
  0, 4095,
  0, endPos
);

if (abs(target - manualTarget) > MANUAL_DEADBAND) {
  manualTarget = target;
  stepper->moveTo(manualTarget);
}
```

Filtering and a deadband were added to reduce target hunting caused by ADC noise.

## Wi-Fi browser control

The XIAO ESP32-S3 creates its own local Wi-Fi network and hosts the control page directly. No external router or Internet connection is required.

### Connection details

```text
Wi-Fi network / SSID: RowingController
Password:             12345678
Controller address:   192.168.4.1
Browser URL:          http://192.168.4.1
```

### Connecting from a phone, tablet or computer

1. Power the rowing-machine controller.
2. Open the Wi-Fi settings on the phone, tablet or computer.
3. Select **RowingController**.
4. Enter the password **12345678**.
5. The device may report that this Wi-Fi network has **no Internet connection**. This is expected; remain connected to it.
6. Open a web browser.
7. Enter **http://192.168.4.1** in the browser address bar.
8. The Rowing Controller web interface should appear.

The address is the ESP32 SoftAP's local address. The browser communicates directly with the S3 rather than through the Internet.

### Web controls

- **Speed 1-5** — selects the automatic sweep speed.
- **STOP** — immediately stops the current automatic movement and disables automatic running.
- **RESUME** — re-enables automatic running.
- **HOME / CENTER** — disables automatic running and commands the carriage to the calibrated midpoint, `endPos / 2`.

### Firmware setup

The access point is defined by:

```cpp
const char* AP_SSID     = "RowingController";
const char* AP_PASSWORD = "12345678";

WebServer server(80);
```

The wireless-limit firmware uses:

```cpp
WiFi.mode(WIFI_AP_STA);
WiFi.softAP(AP_SSID, AP_PASSWORD);
```

`WIFI_AP_STA` is used because the main S3 must simultaneously:

- host the local Wi-Fi control network; and
- operate ESP-NOW to receive the battery-powered HOME and END limit nodes.

The original wired-limit firmware does not require ESP-NOW and therefore uses:

```cpp
WiFi.mode(WIFI_AP);
WiFi.softAP(AP_SSID, AP_PASSWORD);
```

Both firmware variants expose the browser interface at **http://192.168.4.1**.

### Web-server routes

```cpp
server.on("/", handleRoot);
server.on("/speed", handleSpeed);
server.on("/stop", handleStop);
server.on("/resume", handleResume);
server.on("/home", handleHome);
server.begin();
```

The main loop must continuously service browser requests:

```cpp
server.handleClient();
```

### Troubleshooting

If the page does not open:

- Confirm the phone/computer is still connected to **RowingController** rather than automatically switching back to another Wi-Fi network.
- Enter **http://192.168.4.1** explicitly; HTTPS is not used by the controller.
- Ignore the operating system's **No Internet** warning for this network.
- Power-cycle the S3 if the `RowingController` SSID is not visible.
- For the wireless-limit firmware, if the web interface works but ESP-NOW limits stop responding, troubleshoot Wi-Fi/ESP-NOW coexistence separately rather than changing the calibration logic.

## Wireless HOME / END switches

Each endpoint has a XIAO ESP32-C3 and mechanical microswitch.

### Switch wiring

The mechanical microswitch has three terminals: **C / COM (common)**, **NC (normally closed)** and **NO / O (normally open)**.

For the wireless limit nodes, the tested wiring is:

```text
C / COM -> GND
NC      -> GPIO2 signal input on the XIAO ESP32-C3
NO / O  -> not connected
```

So, viewed functionally:

- **C / COM = ground**
- **NC = signal**
- **NO / O = unused**

The C3 uses:

```cpp
pinMode(LIMIT_PIN, INPUT_PULLUP);
```

Node IDs:

```cpp
HOME = 1
END  = 2
```

Packet format:

```cpp
struct LimitPacket {
  uint8_t nodeID;
  uint8_t pressed;
};
```

The tested S3 receiver MAC was:

```text
DC:B4:D9:39:33:C0
```

## Wireless-node battery strategy

The first diagnostic firmware transmitted approximately every 20 ms. That was responsive but wasteful on battery.

The current low-power strategy is:

- Send immediately whenever the switch state changes.
- Send a heartbeat every 5 seconds.
- Disable Bluetooth because it is unused.
- Reduce Wi-Fi transmit power where range allows.
- Remove Serial logging in the finished node.
- Avoid deep sleep for now because it complicated USB/programming during development.

Current optional power-saving lines:

```cpp
btStop();
WiFi.setTxPower(WIFI_POWER_8_5dBm);
```

Reducing packet frequency alone does not eliminate Wi-Fi idle current. For much longer runtime, future versions could investigate modem sleep or light sleep while preserving reliable switch wake-up.

## Key development decisions

- Kept STEP and DIR on GPIO4 and GPIO5 after a basic pulse test proved that path reliable.
- Replaced troublesome long wired limit-switch runs with ESP-NOW wireless nodes.
- Increased C3 transmission rate from 500 ms to 20 ms during debugging after short switch presses were missed.
- Replaced continuous 20 ms transmission with event-driven packets plus heartbeat for battery operation.
- Increased acceleration from 8000 to 17000 so the carriage reaches useful speed earlier in the stroke.
- Replaced analog AUTO speed selection with five fixed web speed levels.
- Fixed manual wandering by preventing the READY state from continuously commanding center.
- Temporarily disabled SoftAP during ESP-NOW debugging, then restored a combined AP/STA architecture.

## Repository structure

```text
Rowing-Machine/
├── README.md
└── src/
    ├── main_controller/
    │   └── main_controller.ino
    ├── main_controller_wired_limits/
    │   └── main_controller_wired_limits.ino
    └── wireless_limit/
        └── wireless_limit.ino
```

## Firmware variants

### Wireless limits — current architecture

`src/main_controller/main_controller.ino`

Uses ESP-NOW packets from the two battery-powered XIAO ESP32-C3 limit nodes.

### Wired limits — original architecture

`src/main_controller_wired_limits/main_controller_wired_limits.ino`

Uses the original direct S3 limit inputs:

```text
HOME -> GPIO43
END  -> GPIO8
```

The wired switches use `INPUT_PULLUP` and are treated as active LOW. This version does not initialize ESP-NOW; it uses `WIFI_AP` only for the browser interface. The calibration, AUTO, MANUAL and web-control structure is otherwise kept aligned with the documented controller.

## Current status

Confirmed during development:

- TB6600 and stepper respond correctly to S3 STEP/DIR control.
- Automatic endpoint calibration works.
- Automatic sweep works.
- Higher acceleration improved stroke behavior.
- Wi-Fi speed levels work.
- STOP, RESUME and center commands were implemented.
- ESP-NOW communication between C3 and S3 was verified.
- A wireless limit node can operate from Li-ion battery power.

## Still to document / refine

- Exact NEMA 17 motor part number.
- Exact rail drive geometry, belt/lead-screw ratio and steps-per-mm.
- TB6600 current and microstep settings.
- Exact 24 V power-supply rating.
- Real battery-current measurements.
- S3-side heartbeat timeout / loss-of-node monitoring.
- Final electrical schematic and enclosure photos.

- [Print Schematic.pdf](https://github.com/user-attachments/files/32558452/Print.Schematic.pdf)

