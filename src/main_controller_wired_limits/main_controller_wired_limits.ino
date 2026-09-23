#include <WiFi.h>
#include <WebServer.h>
#include <FastAccelStepper.h>

// --------------------------------------------------
// PIN ASSIGNMENT
// --------------------------------------------------

const int POT_PIN    = 1;
const int AUTO_PIN   = 2;
const int MANUAL_PIN = 3;
const int STEP_PIN   = 4;
const int DIR_PIN    = 5;

// Original hard-wired limit switch inputs.
const int HOME_PIN   = 43;
const int END_PIN    = 8;

// --------------------------------------------------
// MOTION SETTINGS
// --------------------------------------------------

const uint32_t HOME_SPEED      = 1500;
const uint32_t MAX_SPEED       = 17000;
const uint32_t ACCEL           = 17000;

const uint32_t FAST_HOME_SPEED = 1800;
const uint32_t SLOW_HOME_SPEED = 200;

const int FAST_BACKOFF      = 100;
const int SLOW_SEARCH       = 1000;
const int SEARCH_STEP       = 500;
const int BACKOFF           = 100;
const int POSITION_DEADBAND = 20;
const int MANUAL_DEADBAND   = 50;

// --------------------------------------------------
// WIFI WEB CONTROL
// --------------------------------------------------

const char* AP_SSID     = "RowingController";
const char* AP_PASSWORD = "12345678";

WebServer server(80);

int wifiSpeedLevel = 3;
bool wifiAutoRunning = true;

const uint32_t AUTO_SPEED_1 = 3000;
const uint32_t AUTO_SPEED_2 = 6000;
const uint32_t AUTO_SPEED_3 = 9000;
const uint32_t AUTO_SPEED_4 = 13000;
const uint32_t AUTO_SPEED_5 = 17000;

// --------------------------------------------------
// STEPPER
// --------------------------------------------------

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

int32_t endPos = 0;

enum State {
  FIND_HOME,
  BACKOFF_HOME,
  FIND_HOME_SLOW,
  FIND_END,
  BACKOFF_END,
  FIND_END_SLOW,
  READY
};

State state = FIND_HOME;

// --------------------------------------------------
// HELPERS
// --------------------------------------------------

// Wired switches are assumed to connect the GPIO to GND
// when active and use the ESP32 internal pull-up.
bool homePressed() {
  return digitalRead(HOME_PIN) == LOW;
}

bool endPressed() {
  return digitalRead(END_PIN) == LOW;
}

uint32_t getAutoSpeed() {
  switch (wifiSpeedLevel) {
    case 1: return AUTO_SPEED_1;
    case 2: return AUTO_SPEED_2;
    case 3: return AUTO_SPEED_3;
    case 4: return AUTO_SPEED_4;
    case 5: return AUTO_SPEED_5;
    default: return AUTO_SPEED_3;
  }
}

// --------------------------------------------------
// WEB UI
// --------------------------------------------------

void handleRoot() {
  String html =
    "<!doctype html><html><head>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:Arial,sans-serif;max-width:500px;margin:40px auto;padding:0 20px;}"
    "button,select{font-size:20px;padding:12px;margin:6px 0;width:100%;}"
    "</style></head><body>"
    "<h2>Rowing Controller - Wired Limits</h2>"
    "<form action='/speed'>"
    "<select name='level'>"
    "<option value='1'>Speed 1</option>"
    "<option value='2'>Speed 2</option>"
    "<option value='3'>Speed 3</option>"
    "<option value='4'>Speed 4</option>"
    "<option value='5'>Speed 5</option>"
    "</select>"
    "<button type='submit'>Set speed</button>"
    "</form>"
    "<form action='/stop'><button>STOP</button></form>"
    "<form action='/resume'><button>RESUME</button></form>"
    "<form action='/home'><button>HOME / CENTER</button></form>"
    "</body></html>";

  server.send(200, "text/html", html);
}

void handleSpeed() {
  if (server.hasArg("level")) {
    wifiSpeedLevel = server.arg("level").toInt();

    if (wifiSpeedLevel < 1) wifiSpeedLevel = 1;
    if (wifiSpeedLevel > 5) wifiSpeedLevel = 5;
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop() {
  wifiAutoRunning = false;

  if (stepper)
    stepper->forceStop();

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleResume() {
  wifiAutoRunning = true;

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleHome() {
  wifiAutoRunning = false;

  if (stepper && state == READY)
    stepper->moveTo(endPos / 2);

  server.sendHeader("Location", "/");
  server.send(303);
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(AUTO_PIN, INPUT_PULLUP);
  pinMode(MANUAL_PIN, INPUT_PULLUP);

  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(END_PIN, INPUT_PULLUP);

  // Only the browser AP is needed in the wired version.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/stop", handleStop);
  server.on("/resume", handleResume);
  server.on("/home", handleHome);
  server.begin();

  engine.init();

  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (!stepper) {
    Serial.println("Stepper init failed");
    while (1);
  }

  stepper->setDirectionPin(DIR_PIN);
  stepper->setAcceleration(ACCEL);
  stepper->setSpeedInHz(FAST_HOME_SPEED);

  // Begin HOME search.
  stepper->move(-100000);
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------

void loop() {
  server.handleClient();

  // ----------------------------------------------
  // CALIBRATION STATE MACHINE
  // ----------------------------------------------

  switch (state) {

    case FIND_HOME:
      if (homePressed()) {
        stepper->forceStop();
        stepper->setSpeedInHz(HOME_SPEED);
        stepper->move(FAST_BACKOFF);
        state = BACKOFF_HOME;
      }
      break;

    case BACKOFF_HOME:
      if (!stepper->isRunning()) {
        stepper->setSpeedInHz(SLOW_HOME_SPEED);
        stepper->move(-SLOW_SEARCH);
        state = FIND_HOME_SLOW;
      }
      break;

    case FIND_HOME_SLOW:
      if (homePressed()) {
        stepper->forceStop();
        stepper->setSpeedInHz(HOME_SPEED);
        stepper->move(BACKOFF);

        while (stepper->isRunning()) {
          server.handleClient();
          delay(1);
        }

        stepper->setCurrentPosition(0);

        stepper->setSpeedInHz(FAST_HOME_SPEED);
        stepper->move(100000);
        state = FIND_END;
      }
      break;

    case FIND_END:
      if (endPressed()) {
        stepper->forceStop();
        stepper->setSpeedInHz(HOME_SPEED);
        stepper->move(-FAST_BACKOFF);
        state = BACKOFF_END;
      }
      break;

    case BACKOFF_END:
      if (!stepper->isRunning()) {
        stepper->setSpeedInHz(SLOW_HOME_SPEED);
        stepper->move(SLOW_SEARCH);
        state = FIND_END_SLOW;
      }
      break;

    case FIND_END_SLOW:
      if (endPressed()) {
        stepper->forceStop();

        endPos = stepper->getCurrentPosition();

        stepper->setSpeedInHz(HOME_SPEED);
        stepper->move(-BACKOFF);

        state = READY;
      }
      break;

    case READY: {
      static bool centerMoveDone = false;

      if (!centerMoveDone && !stepper->isRunning()) {
        stepper->moveTo(endPos / 2);
        centerMoveDone = true;
      }

      break;
    }
  }

  if (state != READY)
    return;

  bool autoMode   = (digitalRead(AUTO_PIN) == LOW);
  bool manualMode = (digitalRead(MANUAL_PIN) == LOW);

  // ----------------------------------------------
  // AUTO MODE
  // ----------------------------------------------

  if (autoMode && !manualMode && wifiAutoRunning) {
    stepper->setSpeedInHz(getAutoSpeed());

    if (!stepper->isRunning()) {
      if (stepper->getCurrentPosition() < endPos / 2)
        stepper->moveTo(endPos);
      else
        stepper->moveTo(0);
    }
  }

  // ----------------------------------------------
  // MANUAL MODE
  // ----------------------------------------------

  else if (manualMode && !autoMode) {
    static int32_t manualTarget = -999999;
    static float filteredManualPot = 0;

    filteredManualPot =
      filteredManualPot * 0.90f +
      analogRead(POT_PIN) * 0.10f;

    int32_t target = map(
      (int)filteredManualPot,
      0,
      4095,
      0,
      endPos
    );

    stepper->setSpeedInHz(MAX_SPEED);

    if (abs(target - manualTarget) > MANUAL_DEADBAND) {
      manualTarget = target;
      stepper->moveTo(manualTarget);
    }
  }
}
