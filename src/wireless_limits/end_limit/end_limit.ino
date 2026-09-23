#include <WiFi.h>
#include <esp_now.h>

const int LIMIT_PIN = 2;

// S3 receiver MAC used during development:
uint8_t receiverMAC[] = {
  0xDC, 0xB4, 0xD9, 0x39, 0x33, 0xC0
};

struct LimitPacket {
  uint8_t nodeID;
  uint8_t pressed;
};

LimitPacket packet;

// END wireless limit node.
const uint8_t NODE_ID = 2;  // END

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  // Unused Bluetooth radio off.
  btStop();

  // Reduced TX power for battery life.
  // Increase again if installed range/reliability is insufficient.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    while (1);
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");
    while (1);
  }

  packet.nodeID = NODE_ID;
}

void sendState(bool state) {
  packet.pressed = state;

  esp_now_send(
    receiverMAC,
    (uint8_t *)&packet,
    sizeof(packet)
  );
}

void loop() {
  static bool initialized = false;
  static bool lastState = false;
  static unsigned long lastHeartbeat = 0;

  // With the tested COM->GND / NC->GPIO2 wiring,
  // HIGH corresponded to the desired "pressed" state.
  bool currentState = (digitalRead(LIMIT_PIN) == HIGH);

  // Send initial state once after startup.
  if (!initialized) {
    initialized = true;
    lastState = currentState;
    sendState(currentState);
    lastHeartbeat = millis();
  }

  // Immediate packet on switch change.
  if (currentState != lastState) {
    lastState = currentState;
    sendState(currentState);
    lastHeartbeat = millis();
  }

  // Five-second liveness heartbeat.
  if (millis() - lastHeartbeat >= 5000) {
    sendState(currentState);
    lastHeartbeat = millis();
  }
}
