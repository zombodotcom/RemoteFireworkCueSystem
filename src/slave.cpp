#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG_MODE false
#define MAX_CHANNELS 4
#define EEPROM_ADDR_ARMED 0

#define ARM_SWITCH D3    // Safe input pin for arming switch

#define LED_PIN D7       // NeoPixel strip pin
#define LED_COUNT 6
#define LED_BRIGHTNESS 50

const uint8_t RELAY_PINS[MAX_CHANNELS] = { D1, D2, D5, D6 };

// LED strip mapping
const uint8_t LED_INDEX_FOR_CHANNEL[MAX_CHANNELS] = { 0, 1, 2, 3 };
#define ARM_LED_INDEX 4
#define POWER_LED_INDEX 5

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Master MAC address
uint8_t masterMac[6] = { 0x08, 0x3A, 0x8D, 0xCF, 0xC0, 0x56 };

bool armed = DEBUG_MODE ? true : false;
volatile bool fireTriggerReceived = false;
uint32_t lastProcessedFireID = 0;
uint8_t lastSenderMac[6];

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

CommandPacket incomingFireData;

// ========================== CRC ====================================
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

// ========================= EEPROM ================================
void saveArmState(bool state) {
  EEPROM.write(EEPROM_ADDR_ARMED, state ? 1 : 0);
  EEPROM.commit();
}
bool loadArmState() {
  return EEPROM.read(EEPROM_ADDR_ARMED) == 1;
}

// ========================= LED Helpers ================================
void updateStatusLEDs() {
  for (int i = 0; i < MAX_CHANNELS; i++) {
    pixels.setPixelColor(LED_INDEX_FOR_CHANNEL[i], 0); // Off by default
  }
  pixels.setPixelColor(ARM_LED_INDEX, armed ? pixels.Color(0, 0, 255) : pixels.Color(255, 0, 0));
  pixels.setPixelColor(POWER_LED_INDEX, pixels.Color(50, 50, 50)); // Dim white power indicator
  pixels.show();
}

void flashChannelLED(uint8_t channel) {
  pixels.setPixelColor(LED_INDEX_FOR_CHANNEL[channel], pixels.Color(255, 255, 0)); // Yellow flash
  pixels.show();
  delay(200);
  pixels.setPixelColor(LED_INDEX_FOR_CHANNEL[channel], 0);
  pixels.show();
}

// ========================= ESP-NOW CALLBACK ================================
void onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len < sizeof(CommandPacket)) return;
  memcpy(&incomingFireData, data, sizeof(CommandPacket));
  memcpy(lastSenderMac, mac, 6);
  fireTriggerReceived = true;
}

// ========================= SETUP ================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n🔌 Booting SINGLE-SLAVE SSR Controller (4 channels + LED Strip)");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Initialize relays as OFF (active-low)
  for (int i = 0; i < MAX_CHANNELS; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
  }
    
  pinMode(ARM_SWITCH, INPUT_PULLUP);
  EEPROM.begin(4);
  armed = DEBUG_MODE ? true : loadArmState();

  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  pixels.clear();
  pixels.show();

  updateStatusLEDs();

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_add_peer(masterMac, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);
  esp_now_register_recv_cb(onReceive);

  Serial.println("📡 SINGLE-SLAVE ready!");
  Serial.println("✅ All relays OFF, LEDs ready.");
}

// ========================= LOOP ================================
void loop() {
  bool switchState = DEBUG_MODE ? true : (digitalRead(ARM_SWITCH) == LOW);
  if (switchState != armed) {
    armed = switchState;
    saveArmState(armed);
    Serial.printf("🔔 Arming switch: %s\n", armed ? "true" : "false");
    updateStatusLEDs();
  }

  if (fireTriggerReceived) {
    fireTriggerReceived = false;

    if (!armed) {
      Serial.println("⚠️ Ignored FIRE - disarmed!");
      return;
    }

    if (incomingFireData.crc != calculateCRC(incomingFireData)) {
      Serial.println("❌ CRC mismatch!");
      return;
    }

    if (strcmp(incomingFireData.cmd, "FIRE") != 0) {
      Serial.println("❌ Unknown command!");
      return;
    }

    if (incomingFireData.id == lastProcessedFireID) {
      Serial.println("⚠️ Duplicate FIRE ID ignored.");
      return;
    }

    lastProcessedFireID = incomingFireData.id;

    uint8_t channel = incomingFireData.targetChannel;
    if (channel >= MAX_CHANNELS) {
      Serial.printf("❌ Invalid channel %u\n", channel);
      return;
    }

    Serial.printf("🔥 Activating RELAY %u (LOW=ON)\n", channel);
    digitalWrite(RELAY_PINS[channel], LOW);
    flashChannelLED(channel);
    delay(500);
    digitalWrite(RELAY_PINS[channel], HIGH);
    updateStatusLEDs();

    AckPacket ack = { MSG_ACK, incomingFireData.id, "IGNITED OK", 1, millis() };
    ack.crc = calculateCRC(ack);
    esp_now_send(lastSenderMac, (uint8_t *)&ack, sizeof(ack));
    Serial.printf("✅ Sent ACK for msgID %u\n", ack.responseToID);
  }
}
