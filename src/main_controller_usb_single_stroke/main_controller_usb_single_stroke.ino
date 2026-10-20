#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastAccelStepper.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// XIAO ESP32-S3 / TB6600 / WIRED switches.
// Diagnostic-first: power-on NEVER starts motor. Send CAL to home.
// Example: STARTMM 500 200 means 500 mm/s for 200 mm of travel (if rail permits).
// Firmware still requires an independent hardware power cut / travel protection.

// GPIOs retained from the known working sketch.
const int AUTO_PIN = 2;
const int MANUAL_PIN = 3;
const int STEP_PIN = 4;
const int DIR_PIN = 5;
const int HOME_PIN = 43;
const int END_PIN = 8;

// COM + NC to GND -> false (released=LOW, pressed=HIGH).
// COM + NO to GND -> true  (released=HIGH, pressed=LOW).
// CHECK with STATUS with TB6600 motor supply OFF before allowing motion.
const bool LIMIT_ACTIVE_LOW = false;

// Provisional distance calibration: with the previous 14.285714 steps/mm,
// STARTMM 250 100 produced 118 mm measured physical travel.
// Corrected value: 14.285714 * (100 / 118) = 12.106537 steps/mm.
// Verify over several runs and speeds before gait experiments.
const float STEPS_PER_MM = 12.106537f;
const uint32_t MAX_SPEED_HZ = 17000;
const uint32_t CAL_ACCEL_STEPS_S2 = 17000;
// Fast-but-finite acceleration for the high-speed stroke. Tune against missed steps.
// At 500 mm/s, nominal ramp lasts ~0.121 s and covers ~30.3 mm.
const uint32_t STROKE_ACCEL_STEPS_S2 = 50000;
const uint32_t GAIT_TARGET_MM_S = 500;
const uint32_t MAX_COMMAND_MM_S = 500;
const uint32_t CAL_SPEED_HZ = 300;     // slow first calibration
const uint32_t BACKOFF_SPEED_HZ = 200;
const uint32_t RETURN_SPEED_HZ = 500;
const uint32_t JOG_SPEED_HZ = 200;

// 6000 steps ~= 496 mm using the provisional 12.107 steps/mm, NOT verified safe travel.
// Set this to a safe, mechanically appropriate bound before using CAL.
const int32_t SEARCH_LIMIT_STEPS = 10000;
const int32_t HOME_BACKOFF_STEPS = 300;
const int32_t END_BACKOFF_STEPS = 300;
const int32_t START_MARGIN_STEPS = 100;  // ~8.3 mm using provisional calibration
const int32_t END_MARGIN_STEPS = 100;    // ~8.3 mm using provisional calibration
const int32_t MAX_JOG_STEPS = 200;
const uint32_t SWITCH_SETTLE_MS = 30;
const uint32_t GAIT_WATCHDOG_MS = 500;

const char *AP_SSID = "RowingController";
const char *AP_PASSWORD = "12345678";
WebServer server(80);

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

enum ControllerState {
  UNCALIBRATED,
  SEEK_HOME,
  WAIT_HOME_STOP,
  BACKOFF_HOME,
  FIND_END_SWITCH,
  WAIT_END_STOP,
  BACKOFF_END,
  RETURN_START,
  READY,
  FAULT
};
ControllerState state = UNCALIBRATED;

enum MotionMode { NONE, JOGGING, POSITIONING, STROKE, RESETTING };
MotionMode mode = NONE;
const char *faultReason = "none";
int32_t endPos = 0;
int32_t commandedTarget = 0;
int motionDirection = 0;  // -1 toward HOME; +1 toward END
bool heartbeatRequired = false;
uint32_t lastHeartbeatMs = 0;
uint32_t stopTimeMs = 0;
char inputLine[80];
size_t inputLength = 0;
bool lineOverflow = false;

bool homePressed() { return (digitalRead(HOME_PIN) == LOW) == LIMIT_ACTIVE_LOW; }
bool endPressed() { return (digitalRead(END_PIN) == LOW) == LIMIT_ACTIVE_LOW; }
bool switchCentered() {
  return digitalRead(AUTO_PIN) == HIGH && digitalRead(MANUAL_PIN) == HIGH;
}

const char *stateName() {
  switch (state) {
    case UNCALIBRATED: return "UNCALIBRATED";
    case SEEK_HOME: return "SEEK_HOME";
    case WAIT_HOME_STOP: return "WAIT_HOME_STOP";
    case BACKOFF_HOME: return "BACKOFF_HOME";
    case FIND_END_SWITCH: return "FIND_END_SWITCH";
    case WAIT_END_STOP: return "WAIT_END_STOP";
    case BACKOFF_END: return "BACKOFF_END";
    case RETURN_START: return "RETURN_START";
    case READY: return "READY";
    case FAULT: return "FAULT";
  }
  return "UNKNOWN";
}
const char *modeName() {
  switch (mode) {
    case NONE: return "IDLE";
    case JOGGING: return "JOG";
    case POSITIONING: return "MOVE";
    case STROKE: return "STROKE";
    case RESETTING: return "RESET";
  }
  return "UNKNOWN";
}

void printStatus() {
  Serial.printf("OK state=%s motion=%s pos=%ld end=%ld target=%ld moving=%d home=%d endsw=%d idle=%d fault=%s steps_per_mm=%.4f gait_mm_s=%lu\n",
                stateName(), modeName(),
                (long)(stepper ? stepper->getCurrentPosition() : 0),
                (long)endPos, (long)commandedTarget,
                (stepper && stepper->isRunning()) ? 1 : 0,
                homePressed() ? 1 : 0, endPressed() ? 1 : 0,
                switchCentered() ? 1 : 0, faultReason, STEPS_PER_MM, (unsigned long)GAIT_TARGET_MM_S);
}

void faultStop(const char *reason) {
  if (stepper) stepper->forceStop();
  heartbeatRequired = false;
  mode = NONE;
  motionDirection = 0;
  faultReason = reason;
  state = FAULT;
  Serial.print("EVENT fault=");
  Serial.println(reason);
}

void commandRelative(int32_t displacement, uint32_t speed) {
  int32_t current = stepper->getCurrentPosition();
  commandedTarget = current + displacement;
  motionDirection = displacement > 0 ? 1 : (displacement < 0 ? -1 : 0);
  stepper->setAcceleration(CAL_ACCEL_STEPS_S2);
  stepper->setSpeedInHz(speed);
  stepper->move(displacement);
}
void commandAbsolute(int32_t target, uint32_t speed, MotionMode nextMode, bool hb) {
  int32_t current = stepper->getCurrentPosition();
  commandedTarget = target;
  motionDirection = target > current ? 1 : (target < current ? -1 : 0);
  heartbeatRequired = hb;
  lastHeartbeatMs = millis();
  mode = nextMode;
  stepper->setAcceleration(nextMode == STROKE ? STROKE_ACCEL_STEPS_S2 : CAL_ACCEL_STEPS_S2);
  stepper->setSpeedInHz(speed);
  stepper->moveTo(target);
}

bool readyForUsbMovement() {
  if (state != READY || !stepper) { Serial.println("ERR not_ready"); return false; }
  if (!switchCentered()) { Serial.println("ERR switch_not_centered"); return false; }
  if (stepper->isRunning() || mode != NONE) { Serial.println("ERR busy"); return false; }
  if (homePressed() || endPressed()) { Serial.println("ERR limit_active"); return false; }
  return true;
}

void startCalibration() {
  if (!stepper || stepper->isRunning() || mode != NONE) {
    Serial.println("ERR busy"); return;
  }
  if (!switchCentered()) { Serial.println("ERR switch_not_centered"); return; }
  if (homePressed() || endPressed()) {
    Serial.println("ERR release_both_limits_before_CAL"); return;
  }
  endPos = 0;
  faultReason = "none";
  heartbeatRequired = false;
  state = SEEK_HOME;
  commandRelative(-SEARCH_LIMIT_STEPS, CAL_SPEED_HZ);
  Serial.println("OK calibration_started");
  Serial.println("EVENT seeking_home");
}

void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (strcmp(line, "STATUS") == 0) { printStatus(); return; }
  if (strcmp(line, "HELP") == 0) {
    Serial.println("OK commands=STATUS,HELP,JOG_delta,CAL,MOVE_targetSteps_speedStepsS,RESET,START_speedStepsS_distanceSteps,RUN_speedStepsS_distanceSteps,STARTMM_speedMmS_distanceMm,RUNMM_speedMmS_distanceMm,HB,STOP");
    return;
  }
  if (strcmp(line, "STOP") == 0) {
    // Immediate software stop: coordinates after a fault/stop cannot be trusted.
    faultStop("user_stop");
    Serial.println("OK stopped_recalibrate");
    return;
  }
  if (strcmp(line, "HB") == 0) {
    if (state != READY || mode != STROKE || !heartbeatRequired) {
      Serial.println("ERR no_heartbeat_run"); return;
    }
    lastHeartbeatMs = millis();
    Serial.println("OK hb");
    return;
  }
  if (strcmp(line, "CAL") == 0) { startCalibration(); return; }

  long a = 0, b = 0;
  char extra = 0;
  if (strncmp(line, "JOG ", 4) == 0) {
    if (sscanf(line + 4, "%ld %c", &a, &extra) != 1 ||
        a == 0 || a < -MAX_JOG_STEPS || a > MAX_JOG_STEPS) {
      Serial.println("ERR JOG_range_minus200_to_plus200"); return;
    }
    if (state != UNCALIBRATED || !switchCentered() ||
        stepper->isRunning() || homePressed() || endPressed()) {
      Serial.println("ERR JOG_only_when_uncalibrated_idle_clear_limits"); return;
    }
    mode = JOGGING;
    commandRelative((int32_t)a, JOG_SPEED_HZ);
    Serial.println("OK jog_started");
    return;
  }
  if (strcmp(line, "RESET") == 0) {
    if (!readyForUsbMovement()) return;
    commandAbsolute(START_MARGIN_STEPS, RETURN_SPEED_HZ, RESETTING, false);
    Serial.println("OK reset_started");
    return;
  }

  // Existing commands use raw step pulses and pulses/second.
  // New STARTMM/RUNMM commands use mm/s and mm, to match leg trajectory units.
  bool isStartMm = strncmp(line, "STARTMM ", 8) == 0;
  bool isRunMm = strncmp(line, "RUNMM ", 6) == 0;
  bool isMove = strncmp(line, "MOVE ", 5) == 0;
  bool isStart = strncmp(line, "START ", 6) == 0;
  bool isRun = strncmp(line, "RUN ", 4) == 0;
  if (isStartMm || isRunMm || isMove || isStart || isRun) {
    const char *args = line + (isStartMm ? 8 : isRunMm ? 6 : isMove ? 5 : isStart ? 6 : 4);
    if (sscanf(args, "%ld %ld %c", &a, &b, &extra) != 2) {
      Serial.println("ERR syntax"); return;
    }
    if (!readyForUsbMovement()) return;
    bool inMm = isStartMm || isRunMm;
    bool stroke = !isMove;
    int32_t current = stepper->getCurrentPosition();
    int64_t requested;
    long speed;
    if (inMm) {
      // Intentionally limit experimental carriage speed to 500 mm/s.
      if (a < 1 || a > (long)MAX_COMMAND_MM_S || b <= 0 || b > 100000) {
        Serial.println("ERR mm_speed_or_distance_range"); return;
      }
      speed = lroundf(a * STEPS_PER_MM);
      requested = (int64_t)current + (int64_t)lroundf(b * STEPS_PER_MM);
    } else {
      speed = isMove ? b : a;
      requested = isMove ? (int64_t)a : (int64_t)current + b;
    }
    if (speed < 1 || speed > (long)MAX_SPEED_HZ) {
      Serial.println("ERR invalid_speed"); return;
    }
    if (stroke && (abs(current - START_MARGIN_STEPS) > 3 || b <= 0)) {
      Serial.println("ERR reset_to_start_first"); return;
    }
    // A fast moving carriage needs room to stop if motion must be interrupted.
    // This is a conservative guard for a software-controlled stepper; it is NOT
    // a substitute for physical limit protection or an independent power cut.
    uint32_t motionAccel = stroke ? STROKE_ACCEL_STEPS_S2 : CAL_ACCEL_STEPS_S2;
    int64_t brakingSteps = ((int64_t)speed * speed + 2 * (int64_t)motionAccel - 1)
                           / (2 * (int64_t)motionAccel);
    const int64_t extraClearanceSteps = 150; // ~12.4 mm at provisional scale
    int64_t minPosition = START_MARGIN_STEPS;
    int64_t maxPosition = (int64_t)endPos - END_MARGIN_STEPS;
    if (requested > current) maxPosition -= brakingSteps + extraClearanceSteps;
    if (requested < current) minPosition += brakingSteps + extraClearanceSteps;
    if (requested < minPosition || requested > maxPosition) {
      Serial.printf("ERR insufficient_travel_safe_range=%ld..%ld requested=%ld\n",
                    (long)minPosition, (long)maxPosition, (long)requested);
      return;
    }
    commandAbsolute((int32_t)requested, (uint32_t)speed,
                    stroke ? STROKE : POSITIONING, isRun || isRunMm);
    Serial.printf("OK %s speed_steps_s=%ld target_steps=%ld accel_steps_s2=%lu\n",
                  stroke ? "stroke_started" : "move_started", speed, (long)requested,
                  (unsigned long)(stroke ? STROKE_ACCEL_STEPS_S2 : CAL_ACCEL_STEPS_S2));
    return;
  }
  Serial.println("ERR unknown_command");
}

void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) Serial.println("ERR line_too_long");
      else if (inputLength > 0) {
        inputLine[inputLength] = 0;
        handleCommand(inputLine);
      }
      inputLength = 0;
      lineOverflow = false;
    } else if (!lineOverflow) {
      if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
      else lineOverflow = true;
    }
  }
}

void serviceHoming() {
  if (!stepper) return;
  if (!switchCentered()) { faultStop("toggle_not_centered_during_calibration"); return; }
  if (homePressed() && endPressed()) { faultStop("both_limits_pressed"); return; }
  switch (state) {
    case SEEK_HOME:
      if (endPressed()) { faultStop("unexpected_end_during_home"); return; }
      if (homePressed()) {
        stepper->forceStop();
        stopTimeMs = millis();
        state = WAIT_HOME_STOP;
        Serial.printf("EVENT home_touched pos=%ld\n", (long)stepper->getCurrentPosition());
      } else if (!stepper->isRunning()) faultStop("home_not_found");
      break;
    case WAIT_HOME_STOP:
      if (!stepper->isRunning() && millis() - stopTimeMs >= SWITCH_SETTLE_MS) {
        if (!homePressed()) { faultStop("home_contact_unstable"); return; }
        commandRelative(HOME_BACKOFF_STEPS, BACKOFF_SPEED_HZ);
        state = BACKOFF_HOME;
        Serial.println("EVENT home_backoff_started_positive");
      }
      break;
    case BACKOFF_HOME:
      if (!stepper->isRunning()) {
        Serial.printf("EVENT home_backoff_done pos=%ld home=%d\n",
                      (long)stepper->getCurrentPosition(), homePressed());
        if (homePressed()) { faultStop("home_not_released"); return; }
        stepper->setCurrentPosition(0);
        commandRelative(SEARCH_LIMIT_STEPS, CAL_SPEED_HZ);
        state = FIND_END_SWITCH;
        Serial.println("EVENT seeking_end_positive");
      }
      break;
    case FIND_END_SWITCH:
      if (homePressed()) { faultStop("home_active_while_seeking_end"); return; }
      if (endPressed()) {
        stepper->forceStop();
        stopTimeMs = millis();
        state = WAIT_END_STOP;
        Serial.printf("EVENT end_touched pos=%ld\n", (long)stepper->getCurrentPosition());
      } else if (!stepper->isRunning()) faultStop("end_not_found");
      break;
    case WAIT_END_STOP:
      if (!stepper->isRunning() && millis() - stopTimeMs >= SWITCH_SETTLE_MS) {
        if (!endPressed()) { faultStop("end_contact_unstable"); return; }
        commandRelative(-END_BACKOFF_STEPS, BACKOFF_SPEED_HZ);
        state = BACKOFF_END;
        Serial.println("EVENT end_backoff_started_negative");
      }
      break;
    case BACKOFF_END:
      if (!stepper->isRunning()) {
        Serial.printf("EVENT end_backoff_done pos=%ld endsw=%d\n",
                      (long)stepper->getCurrentPosition(), endPressed());
        if (endPressed()) { faultStop("end_not_released"); return; }
        endPos = stepper->getCurrentPosition();
        if (endPos <= START_MARGIN_STEPS + END_MARGIN_STEPS + 100) {
          faultStop("calibrated_travel_too_short"); return;
        }
        commandAbsolute(START_MARGIN_STEPS, RETURN_SPEED_HZ, RESETTING, false);
        state = RETURN_START;
        Serial.printf("EVENT travel_calibrated end=%ld returning_start\n", (long)endPos);
      }
      break;
    case RETURN_START:
      if (homePressed()) { faultStop("home_during_return"); return; }
      if (!stepper->isRunning()) {
        mode = NONE;
        motionDirection = 0;
        state = READY;
        Serial.println("EVENT ready_at_start");
      }
      break;
    default: break;
  }
}

void serviceMovement() {
  if (!stepper) return;
  bool moving = stepper->isRunning();
  if (moving) {
    // Directional protection. Do not fault on a switch while moving AWAY from it.
    if ((motionDirection < 0 && homePressed()) ||
        (motionDirection > 0 && endPressed())) {
      faultStop("limit_during_movement"); return;
    }
    if (!switchCentered()) { faultStop("toggle_not_centered"); return; }
    if (state == READY && heartbeatRequired &&
        (uint32_t)(millis() - lastHeartbeatMs) > GAIT_WATCHDOG_MS) {
      faultStop("heartbeat_timeout"); return;
    }
  } else if (mode != NONE) {
    if (state == READY) {
      if (mode == STROKE) Serial.println("EVENT stroke_complete");
      else if (mode == RESETTING) Serial.println("EVENT reset_complete");
      else if (mode == POSITIONING) Serial.println("EVENT move_complete");
    } else if (mode == JOGGING) {
      Serial.println("EVENT jog_complete");
    }
    mode = NONE;
    motionDirection = 0;
    heartbeatRequired = false;
  }
}

void handleRoot() {
  server.send(200, "text/html",
              "<html><body><h3>USB Gantry Calibration</h3>"
              "<p>USB serial: STATUS, JOG, CAL, RESET, STARTMM 500 200. "
              "<a href='/stop'>STOP</a></p></body></html>");
}
void handleWebStop() {
  faultStop("web_stop");
  server.send(200, "text/plain", "Stopped; calibrate again with CAL");
}

void setup() {
  Serial.begin(115200);
  pinMode(AUTO_PIN, INPUT_PULLUP);
  pinMode(MANUAL_PIN, INPUT_PULLUP);
  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(END_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  server.on("/", handleRoot);
  server.on("/stop", handleWebStop);
  server.begin();
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    Serial.println("ERR stepper_initialization_failed");
    while (true) delay(100);
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setAcceleration(CAL_ACCEL_STEPS_S2);
  stepper->setSpeedInHz(JOG_SPEED_HZ);
  Serial.println("EVENT boot_no_automatic_motion");
  Serial.println("EVENT verify_switches_then_send_CAL");
}

void loop() {
  server.handleClient();
  serviceSerial();
  if (state == SEEK_HOME || state == WAIT_HOME_STOP || state == BACKOFF_HOME ||
      state == FIND_END_SWITCH || state == WAIT_END_STOP || state == BACKOFF_END ||
      state == RETURN_START) {
    serviceHoming();
    if (state == FAULT) return;
  }
  if (state == READY || state == UNCALIBRATED) serviceMovement();
}