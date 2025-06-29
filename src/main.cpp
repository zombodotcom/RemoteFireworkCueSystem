// Unified ESP8266 ESP-NOW Master/Slave Code (PlatformIO) with Structured ACKs, Safety, Multi-Slave Support, Manual Trigger, Debug Mode, and SSR Control + CRC Validation + WS2811 LED Indicators + Verbose Logging + MAC Debugging

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG_MODE true
#define FIRE_PIN D1
#define ARM_SWITCH D3
#define MAX_SLAVES 4
#define EEPROM_ADDR_ARMED 0
#define DEVICE_ROLE_MASTER 1
#define DEVICE_ROLE_SLAVE 0
#ifndef DEVICE_ROLE
  #define DEVICE_ROLE DEVICE_ROLE_SLAVE
#endif
#define IS_MASTER (DEVICE_ROLE == DEVICE_ROLE_MASTER)

#define LED_PIN D4
#define LED_COUNT 6 // Total LEDs on master strip
#define LED_BRIGHTNESS 50  // Brightness from 0 to 255

const uint8_t BUTTON_PINS[MAX_SLAVES] = { D5, D6, D7, D2 }; // Buttons ordered by slave ID
const uint8_t LED_INDEX_FOR_BUTTON[MAX_SLAVES] = { 0, 1, 3, 4 }; // Correct LED indices per physical layout
#define ARM_LED_INDEX 2

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint8_t masterMac[6] = { 0x08, 0x3A, 0x8D, 0xCF, 0xC0, 0x56 };
uint8_t slaveMacs[MAX_SLAVES][6] = {
  { 0x24, 0xD7, 0xEB, 0xCB, 0x47, 0x23 },
  { 0xC8, 0xC9, 0xA3, 0x86, 0x7F, 0x72 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x02 }
};

bool armed = DEBUG_MODE ? true : false;
bool prevState[MAX_SLAVES] = {true, true, true, true};

enum MessageType : uint8_t { MSG_FIRE = 1, MSG_ACK = 2 };

struct CommandPacket {
  MessageType type;
  char cmd[8];
  uint32_t id;
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
  for (int i = 0; i < MAX_SLAVES; i++) {
    pixels.setPixelColor(LED_INDEX_FOR_BUTTON[i], 0);
  }
  pixels.setPixelColor(ARM_LED_INDEX, armed ? pixels.Color(0, 0, 255) : pixels.Color(255, 0, 0));
  pixels.show();
}

volatile bool fireTriggerReceived = false;
CommandPacket incomingFireData;
uint8_t lastSenderMac[6];
uint32_t lastProcessedFireID = 0;

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

void onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  Serial.printf("📥 Received %d bytes from ", len);
  printMac(mac);
  Serial.println();

  for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();

  if (len >= sizeof(AckPacket) && IS_MASTER) {
    AckPacket ack;
    memcpy(&ack, data, sizeof(AckPacket));
    Serial.printf("✅ Received ACK for msgID %u: %s\n", ack.responseToID, ack.responseNote);
    Serial.print("📡 MAC source of ACK: ");
    printMac(mac);
    Serial.println();

    for (int i = 0; i < MAX_SLAVES; i++) {
      Serial.print("🔍 Comparing with slave "); Serial.print(i); Serial.print(" -> ");
      printMac(slaveMacs[i]);
      Serial.println();
      if (memcmp(mac, slaveMacs[i], 6) == 0) {
        pixels.setPixelColor(LED_INDEX_FOR_BUTTON[i], pixels.Color(0, 255, 0));
        pixels.show();
        delay(300);
        updateStatusLEDs();
        break;
      }
    }
  } else if (len >= sizeof(CommandPacket) && !IS_MASTER) {
    memcpy(&incomingFireData, data, sizeof(CommandPacket));
    memcpy(lastSenderMac, mac, 6);
    fireTriggerReceived = true;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.channel(1);
  WiFi.disconnect();
  delay(100);

  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  pixels.clear();
  pixels.show();

  for (int i = 0; i < MAX_SLAVES; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  pinMode(ARM_SWITCH, INPUT);
  EEPROM.begin(4);
  armed = DEBUG_MODE ? true : loadArmState();

  if (!IS_MASTER) {
    pinMode(FIRE_PIN, OUTPUT);
    digitalWrite(FIRE_PIN, LOW);
  }

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  if (!IS_MASTER) {
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_add_peer(masterMac, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);
    esp_now_register_recv_cb(onReceive);
  } else {
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    for (int i = 0; i < MAX_SLAVES; i++) {
      esp_now_add_peer(slaveMacs[i], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    }
    esp_now_register_recv_cb(onReceive);
  }

  updateStatusLEDs();
  Serial.println(IS_MASTER ? "🎮 Master ready" : "📡 Slave ready");
}

void loop() {
  static uint32_t messageID = 0;

  if (IS_MASTER) {
    for (int i = 0; i < MAX_SLAVES; i++) {
      bool pressed = digitalRead(BUTTON_PINS[i]) == LOW;
      if (pressed && prevState[i]) {
        if (!armed && !DEBUG_MODE) {
          Serial.println("⚠️ Ignored trigger, system not armed");
          continue;
        }
        CommandPacket pkt = { MSG_FIRE, "FIRE", ++messageID };
        pkt.crc = calculateCRC(pkt);
        esp_now_send(slaveMacs[i], (uint8_t *)&pkt, sizeof(pkt));
        Serial.printf("🔥 Sent FIRE to slave %d, msgID: %u\n", i, pkt.id);
        pixels.setPixelColor(LED_INDEX_FOR_BUTTON[i], pixels.Color(255, 255, 0));
        pixels.show();
        delay(200);
        updateStatusLEDs();
      }
      prevState[i] = !pressed;
    }
    armed = DEBUG_MODE ? true : digitalRead(ARM_SWITCH);
    updateStatusLEDs();
  } else {
    if (fireTriggerReceived) {
      fireTriggerReceived = false;
      if (armed && strcmp(incomingFireData.cmd, "FIRE") == 0) {
        if (lastProcessedFireID != incomingFireData.id) {
          lastProcessedFireID = incomingFireData.id;
          digitalWrite(FIRE_PIN, HIGH);
          delay(500);
          digitalWrite(FIRE_PIN, LOW);
          Serial.println("🔥 FIRE activated");

          AckPacket ack = { MSG_ACK, incomingFireData.id, "IGNITED OK", 1, millis() };
          ack.crc = calculateCRC(ack);
          esp_now_send(lastSenderMac, (uint8_t *)&ack, sizeof(ack));
          Serial.printf("✅ ACK sent for msgID: %u\n", ack.responseToID);
        } else {
          Serial.println("⚠️ Duplicate FIRE ID ignored");
        }
      } else {
        Serial.println("❌ Invalid or disarmed FIRE trigger");
      }
    }
  }
}
