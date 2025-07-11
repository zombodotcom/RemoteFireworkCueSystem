#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG_MODE false
#define MAX_CHANNELS 4
#define EEPROM_ADDR_ARMED 0
#define ARM_SWITCH D3

#define LED_PIN D4
#define LED_COUNT 6
#define LED_BRIGHTNESS 50

// Button inputs (matching your board)
const uint8_t BUTTON_PINS[MAX_CHANNELS] = { D5, D6, D7, D2 };

// LED strip layout
const uint8_t LED_INDEX_FOR_BUTTON[MAX_CHANNELS] = { 0, 1, 3, 4 };
#define ARM_LED_INDEX 2

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Slave MAC (ONLY ONE)
uint8_t slaveMac[6] = { 0xA0, 0x20, 0xA6, 0x1B, 0x42, 0x1A };

// Arming and button state
bool armed = DEBUG_MODE ? true : false;
bool prevButtonState[MAX_CHANNELS] = { true, true, true, true };
uint8_t lastFiredChannel = 255;

enum MessageType : uint8_t { MSG_FIRE = 1, MSG_ACK = 2 };

struct CommandPacket {
  MessageType type;
  char cmd[8];
  uint32_t id;
  uint8_t targetChannel;
  uint32_t crc;
};

struct AckPacket {
  MessageType type;
  uint32_t responseToID;
  char responseNote[16];
  uint8_t deviceStatus;
  uint32_t timestamp;
  uint32_t crc;
};

uint32_t messageID = 0;

uint32_t calculateCRC(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  while (length--) {
    uint8_t b = *data++;
    for (int i = 0; i < 8; i++) {
      uint32_t mask = -(crc ^ b) & 1;
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
      b >>= 1;
    }
  }
  return ~crc;
}
uint32_t calculateCRC(const CommandPacket &packet) {
  return calculateCRC((const uint8_t *)&packet, sizeof(packet) - sizeof(packet.crc));
}
uint32_t calculateCRC(const AckPacket &packet) {
  return calculateCRC((const uint8_t *)&packet, sizeof(packet) - sizeof(packet.crc));
}

void saveArmState(bool state) {
  EEPROM.write(EEPROM_ADDR_ARMED, state ? 1 : 0);
  EEPROM.commit();
}
bool loadArmState() {
  return EEPROM.read(EEPROM_ADDR_ARMED) == 1;
}

void updateStatusLEDs() {
  for (int i = 0; i < MAX_CHANNELS; i++) {
    pixels.setPixelColor(LED_INDEX_FOR_BUTTON[i], 0);
  }
  pixels.setPixelColor(ARM_LED_INDEX, armed ? pixels.Color(0, 0, 255) : pixels.Color(255, 0, 0));
  pixels.show();
}

void onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  Serial.println("📥 Received data from slave!");
  if (len < sizeof(AckPacket)) return;
  AckPacket ack;
  memcpy(&ack, data, sizeof(AckPacket));
  if (!DEBUG_MODE && ack.crc != calculateCRC(ack)) {
    Serial.println("❌ CRC mismatch on ACK!");
    return;
  }

  Serial.printf("✅ ACK for msgID %u: %s\n", ack.responseToID, ack.responseNote);

  // Light the *same button* that triggered this fire
  if (lastFiredChannel < MAX_CHANNELS) {
    pixels.setPixelColor(LED_INDEX_FOR_BUTTON[lastFiredChannel], pixels.Color(0, 255, 0));
    pixels.show();
    delay(300);
    updateStatusLEDs();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n🔌 Booting SINGLE-MASTER Controller (1 slave, 4 channels)");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  pixels.clear();
  pixels.show();

  for (int i = 0; i < MAX_CHANNELS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  pinMode(ARM_SWITCH, INPUT);
  EEPROM.begin(4);
  armed = DEBUG_MODE ? true : loadArmState();

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(slaveMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_recv_cb(onReceive);

  updateStatusLEDs();
  Serial.println("🎮 SINGLE-MASTER ready!");
}

void loop() {
  bool switchState = DEBUG_MODE ? true : (digitalRead(ARM_SWITCH) == LOW);
  if (switchState != armed) {
    armed = switchState;
    saveArmState(armed);
    Serial.printf("🔔 Arming switch: %s\n", armed ? "true" : "false");
    updateStatusLEDs();
  }

  for (int i = 0; i < MAX_CHANNELS; i++) {
    bool pressed = (digitalRead(BUTTON_PINS[i]) == LOW);
    if (pressed && prevButtonState[i]) {
      if (!armed && !DEBUG_MODE) {
        Serial.println("⚠️ Ignored FIRE - disarmed!");
        continue;
      }

      CommandPacket pkt = { MSG_FIRE, "FIRE", ++messageID, i };
      pkt.crc = calculateCRC(pkt);

      esp_now_send(slaveMac, (uint8_t *)&pkt, sizeof(pkt));
      Serial.printf("🔥 Sent FIRE to channel %u (msgID: %u)\n", i, pkt.id);

      lastFiredChannel = i;

      // Flash the button LED yellow
      pixels.setPixelColor(LED_INDEX_FOR_BUTTON[i], pixels.Color(255, 255, 0));
      pixels.show();
      delay(200);
      updateStatusLEDs();
    }
    prevButtonState[i] = !pressed;
  }
}
