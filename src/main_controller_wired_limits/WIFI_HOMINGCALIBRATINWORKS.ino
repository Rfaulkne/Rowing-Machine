/*
 RowingController_V8
 XIAO ESP32-S3 + TB6600
 FastAccelStepper 1.2.7

 Toggle:
   UP     = AUTO SWEEP
   CENTER = IDLE
   DOWN   = MANUAL POSITION

 Pins
 ----
 STEP  = GPIO4
 DIR   = GPIO5
 POT   = GPIO1
 AUTO  = GPIO2
 MANUAL= GPIO3
 HOME  = GPIO43
 END   = GPIO8
*/

#include <FastAccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>

const int STEP_PIN=4;
const int DIR_PIN=5;
const int POT_PIN=1;

const int AUTO_PIN=2;
const int MANUAL_PIN=3;

const int HOME_PIN=43;
const int END_PIN=8;

const uint32_t HOME_SPEED=1500; //400 og
const uint32_t MAX_SPEED=17000; //1 2000og
const uint32_t ACCEL=17000;/// was 8000

const uint32_t FAST_HOME_SPEED = 1800;
const uint32_t SLOW_HOME_SPEED = 200;

const int FAST_BACKOFF = 100;
const int SLOW_SEARCH = 1000;

const int SEARCH_STEP=500;
const int BACKOFF=100;
const int POSITION_DEADBAND=20;
const int MANUAL_DEADBAND = 50;

FastAccelStepperEngine engine;
FastAccelStepper *stepper=nullptr;

WebServer server(80);

const char* AP_SSID = "RowingController";
const char* AP_PASSWORD = "12345678";

int wifiSpeedLevel = 3;
bool wifiAutoRunning = true;

const uint32_t AUTO_SPEED_1 = 3000;
const uint32_t AUTO_SPEED_2 = 6000;
const uint32_t AUTO_SPEED_3 = 9000;
const uint32_t AUTO_SPEED_4 = 13000;
const uint32_t AUTO_SPEED_5 = 17000;

enum State{
  FIND_HOME,
  BACKOFF_HOME,
  FIND_HOME_SLOW,

  FIND_END,
  BACKOFF_END,
  FIND_END_SLOW,

  READY
};

State state=FIND_HOME;

int32_t endPos=0;
int32_t lastTarget=-999999;

bool endSwitchSeen = false;

void moveRel(int32_t s){ stepper->move(s); }

void handleRoot() {

  String page = "";

  page += "<html><head>";
  page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "</head><body>";

  page += "<h1>Rowing Controller</h1>";

  page += "<h2>AUTO SPEED</h2>";

  page += "<form action='/speed' method='get'>";

  page += "<select name='level' onchange='this.form.submit()' style='font-size:24px'>";

  for (int i = 1; i <= 5; i++) {

    page += "<option value='";
    page += i;

    if (i == wifiSpeedLevel)
      page += "' selected>";
    else
      page += "'>";

    page += "Speed ";
    page += i;
    page += "</option>";
  }

  page += "</select>";

  page += "</form>";

  page += "<br><br>";

  page += "<form action='/stop'>";
  page += "<button type='submit' style='font-size:24px'>STOP</button>";
  page += "</form>";

  page += "<br>";

  page += "<form action='/resume'>";
  page += "<button type='submit' style='font-size:24px'>RESUME</button>";
  page += "</form>";

  page += "<br>";

  page += "<form action='/home'>";
  page += "<button type='submit' style='font-size:24px'>HOME</button>";
  page += "</form>";

  page += "</body></html>";

  server.send(200, "text/html", page);
}

void handleSpeed() {

  if (server.hasArg("level")) {

    wifiSpeedLevel = server.arg("level").toInt();

    if (wifiSpeedLevel < 1)
      wifiSpeedLevel = 1;

    if (wifiSpeedLevel > 5)
      wifiSpeedLevel = 5;
  }

  server.sendHeader("Location", "/");
  server.send(303);
}


void handleStop() {

  wifiAutoRunning = false;
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

  stepper->moveTo(endPos / 2);

  server.sendHeader("Location", "/");
  server.send(303);
}


uint32_t getAutoSpeed() {

  switch (wifiSpeedLevel) {

    case 1:
      return AUTO_SPEED_1;

    case 2:
      return AUTO_SPEED_2;

    case 3:
      return AUTO_SPEED_3;

    case 4:
      return AUTO_SPEED_4;

    case 5:
      return AUTO_SPEED_5;

    default:
      return AUTO_SPEED_3;
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/stop", handleStop);
  server.on("/resume", handleResume);
  server.on("/home", handleHome);

  server.begin();

  pinMode(HOME_PIN,INPUT_PULLUP);
  pinMode(END_PIN,INPUT_PULLUP);
  pinMode(AUTO_PIN,INPUT_PULLUP);
  pinMode(MANUAL_PIN,INPUT_PULLUP);

  engine.init();

  stepper=engine.stepperConnectToPin(STEP_PIN);
  if(!stepper) while(1);

  stepper->setDirectionPin(DIR_PIN);
  stepper->setAutoEnable(false);
  stepper->setAcceleration(ACCEL);
  stepper->setSpeedInHz(HOME_SPEED);

  //moveRel(-SEARCH_STEP);
  moveRel(-100000);
}

void loop(){

  server.handleClient();
  switch(state){

case FIND_HOME:

  if (digitalRead(HOME_PIN)) {

    stepper->forceStop();

    state = BACKOFF_HOME;
  }

  return;

  case BACKOFF_HOME:

    static bool backoffStarted = false;

    if (!stepper->isRunning()) {

      if (!backoffStarted) {

        moveRel(BACKOFF);

        backoffStarted = true;
      }

      else {

        backoffStarted = false;
          stepper->setSpeedInHz(SLOW_HOME_SPEED);

          moveRel(-SLOW_SEARCH);

          state = FIND_HOME_SLOW;
      }
    }

    return;

    case FIND_HOME_SLOW:

      if (digitalRead(HOME_PIN)) {

        stepper->forceStop();

        if (!stepper->isRunning()) {

          moveRel(FAST_BACKOFF);

          while(stepper->isRunning());

          stepper->setCurrentPosition(0);

          stepper->setSpeedInHz(FAST_HOME_SPEED);

          moveRel(100000);

          state = FIND_END;
       }
    }

    return;

    case FIND_END:

      if (digitalRead(END_PIN)) {

        stepper->forceStop();

        state = BACKOFF_END;
      }

      return;

      case BACKOFF_END:

        static bool endBackoffStarted = false;

        if (!stepper->isRunning()) {

          if (!endBackoffStarted) {

            moveRel(-FAST_BACKOFF);      // <-- FIRST back off from the switch
            endBackoffStarted = true;

          } else {

            endBackoffStarted = false;

            stepper->setSpeedInHz(SLOW_HOME_SPEED);

            moveRel(SLOW_SEARCH);        // <-- NOW creep back onto the switch

            state = FIND_END_SLOW;
          }
        }

      return;

    case FIND_END_SLOW:

      if (digitalRead(END_PIN)) {
        stepper->forceStop();
        endSwitchSeen = true;
      }

      if (endSwitchSeen && !stepper->isRunning()) {

        endSwitchSeen = false;

        endPos = stepper->getCurrentPosition();

        moveRel(-FAST_BACKOFF);

        stepper->setSpeedInHz(HOME_SPEED);

        endSwitchSeen = false;

        state = READY;
  }

  return;

    case READY:

    static bool centerMoveDone = false;

    if (!centerMoveDone) {
      stepper->moveTo(endPos / 2);
      centerMoveDone = true;
    }

    break;
}

  bool autoMode   = digitalRead(AUTO_PIN);
  bool manualMode = digitalRead(MANUAL_PIN);

  if(autoMode && !manualMode && wifiAutoRunning){

  uint32_t s = getAutoSpeed();

  stepper->setSpeedInHz(s);

  if (!stepper->isRunning()) {

    if (stepper->getCurrentPosition() < endPos / 2)
      stepper->moveTo(endPos);
    else
      stepper->moveTo(0);
  }
}

  else if(manualMode && !autoMode){

  static int32_t manualTarget = -999999;
  static float filteredManualPot = 0;

  filteredManualPot = filteredManualPot * 0.90f +
                      analogRead(POT_PIN) * 0.10f;

  int32_t target = map(
    (int)filteredManualPot,
    0,
    4095,
    0,
    endPos
  );

  stepper->setSpeedInHz(MAX_SPEED);

  // Only issue a new move when the requested position
  // has actually changed significantly.
  if (abs(target - manualTarget) > 50) {

    manualTarget = target;

    stepper->moveTo(manualTarget);
  }
}
}

  //else{
    // center position = idle
  //}

  //if(digitalRead(HOME_PIN) && stepper->getCurrentPosition()>200){
  //  stepper->forceStop();
    
 // }

 // if(digitalRead(END_PIN) && stepper->getCurrentPosition()<endPos-200){
 //   stepper->forceStop();
    
 // }

