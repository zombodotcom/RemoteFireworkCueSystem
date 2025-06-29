// Unified ESP8266 ESP-NOW Master/Slave Code (PlatformIO) with Structured ACKs, Safety, Multi-Slave Support, Manual Trigger, Debug Mode, and SSR Control + CRC Validation

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define DEBUG_MODE true
#define FIRE_PIN D1
#define LED_SEND D2
#define LED_RECV D3
#define ARM_SWITCH D5
#define ARM_LED D6
#define TRIGGER_BTN D7
#define MAX_SLAVES 5
#define EEPROM_ADDR_ARMED 0
#define DEVICE_ROLE_MASTER 1
#define DEVICE_ROLE_SLAVE 0
#ifndef DEVICE_ROLE
  #define DEVICE_ROLE DEVICE_ROLE_SLAVE
#endif
#define IS_MASTER (DEVICE_ROLE == DEVICE_ROLE_MASTER)

uint8_t masterMac[6] = { 0x08, 0x3A, 0x8D, 0xCF, 0xC0, 0x56 };
uint8_t slaveMacs[MAX_SLAVES][6] = {
  { 0x24, 0xD7, 0xEB, 0xCB, 0x47, 0x23 },
  { 0xC8, 0xC9, 0xA3, 0x86, 0x7F, 0x72 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x02 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x03 }
};

bool armed = DEBUG_MODE ? true : false;
bool blinkArmed = false;
unsigned long blinkUntil = 0;
bool blinkRecv = false;

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

struct LastResponse {
  uint32_t responseToID;
  char responseNote[16];
  uint8_t status;
  uint32_t timestamp;
};

LastResponse slaveResponses[MAX_SLAVES] = {0};
uint32_t lastProcessedFireID = 0;
uint32_t messageID = 0;
volatile bool fireTriggerReceived = false;
CommandPacket incomingFireData;
uint8_t lastSenderMac[6] = {0};

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

void blinkLED(uint8_t pin, int duration = 200) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
}

void saveArmState(bool state) {
  EEPROM.write(EEPROM_ADDR_ARMED, state ? 1 : 0);
  EEPROM.commit();
}

bool loadArmState() {
  return EEPROM.read(EEPROM_ADDR_ARMED) == 1;
}

void IRAM_ATTR onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  Serial.printf("📥 Received %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n", len,
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();

  if (!data || len < 1) return;
  MessageType type = (MessageType)data[0];

  if (!IS_MASTER && armed && type == MSG_FIRE && len >= sizeof(CommandPacket)) {
    memcpy(&incomingFireData, data, sizeof(CommandPacket));
    if (!DEBUG_MODE && incomingFireData.crc != calculateCRC(incomingFireData)) {
      Serial.println("❌ Invalid CRC on FIRE packet");
      return;
    }
    memcpy(lastSenderMac, mac, 6);
    fireTriggerReceived = true;
  } else if (IS_MASTER && type == MSG_ACK && len >= sizeof(AckPacket)) {
    AckPacket ack;
    memcpy(&ack, data, sizeof(AckPacket));
    if (!DEBUG_MODE && ack.crc != calculateCRC(ack)) {
      Serial.println("❌ Invalid CRC on ACK packet");
      return;
    }
    for (int i = 0; i < MAX_SLAVES; i++) {
      if (memcmp(mac, slaveMacs[i], 6) == 0) {
        if (slaveResponses[i].responseToID == ack.responseToID) {
          Serial.printf("⚠️ Duplicate ACK ID %u from slave %d\n", ack.responseToID, i);
          return;
        }
        slaveResponses[i].responseToID = ack.responseToID;
        strncpy(slaveResponses[i].responseNote, ack.responseNote, sizeof(ack.responseNote));
        slaveResponses[i].status = ack.deviceStatus;
        slaveResponses[i].timestamp = ack.timestamp;

        Serial.printf("✅ ACK ID %u [%s] Status:%d Time:%u from slave %d\n",
          ack.responseToID,
          ack.responseNote,
          ack.deviceStatus,
          ack.timestamp,
          i);
        break;
      }
    }
  } else {
    blinkRecv = true;
  }
}

void checkArming() {
  bool switchState = DEBUG_MODE ? true : digitalRead(ARM_SWITCH);
  if (switchState && !armed) {
    armed = true;
    saveArmState(true);
    Serial.println("🟢 System armed");
  } else if (!switchState && armed) {
    armed = false;
    saveArmState(false);
    Serial.println("🔴 System disarmed");
  }
  digitalWrite(ARM_LED, armed ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.channel(1);
  WiFi.disconnect();
  delay(100);

  pinMode(LED_SEND, OUTPUT);
  pinMode(LED_RECV, OUTPUT);
  pinMode(ARM_LED, OUTPUT);
  digitalWrite(LED_SEND, LOW);
  digitalWrite(LED_RECV, LOW);
  digitalWrite(ARM_LED, LOW);

  pinMode(ARM_SWITCH, INPUT);
  if (IS_MASTER) pinMode(TRIGGER_BTN, INPUT_PULLUP);
  EEPROM.begin(4);
  armed = DEBUG_MODE ? true : loadArmState();
  digitalWrite(ARM_LED, armed ? HIGH : LOW);

  if (!IS_MASTER) {
    pinMode(FIRE_PIN, OUTPUT);
    digitalWrite(FIRE_PIN, LOW);
  }

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  if (!IS_MASTER) {
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    Serial.print("📡 Slave MAC: ");
    Serial.println(WiFi.macAddress());
    esp_now_add_peer(masterMac, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);
  } else {
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    for (int i = 0; i < MAX_SLAVES; i++) {
      esp_now_add_peer(slaveMacs[i], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    }
    Serial.println("🎮 Master ready");
  }
}

void loop() {
  if (IS_MASTER) checkArming();

  if (IS_MASTER) {
    static unsigned long lastSent = 0;
    bool trigger = DEBUG_MODE ? (millis() - lastSent > 5000) : (digitalRead(TRIGGER_BTN) == LOW);
    if (trigger && (millis() - lastSent > 1000)) {
      CommandPacket pkt = { MSG_FIRE, "FIRE", ++messageID };
      pkt.crc = calculateCRC(pkt);
      for (int i = 0; i < MAX_SLAVES; i++) {
        esp_now_send(slaveMacs[i], (uint8_t *)&pkt, sizeof(pkt));
        Serial.printf("📤 Sent to slave %d [%02X:%02X:%02X:%02X:%02X:%02X]\n", i,
          slaveMacs[i][0], slaveMacs[i][1], slaveMacs[i][2],
          slaveMacs[i][3], slaveMacs[i][4], slaveMacs[i][5]);
      }
      Serial.println("🔥 FIRE command sent to all slaves");
      blinkLED(LED_SEND);
      lastSent = millis();
    }
  }

  if (fireTriggerReceived) {
    fireTriggerReceived = false;
    if (armed && strcmp(incomingFireData.cmd, "FIRE") == 0) {
      if (lastProcessedFireID == incomingFireData.id) {
        Serial.println("⚠️ Duplicate FIRE command ignored");
      } else {
        lastProcessedFireID = incomingFireData.id;
        Serial.println("🔥 Valid FIRE received!");

        digitalWrite(FIRE_PIN, HIGH);
        delay(500);
        digitalWrite(FIRE_PIN, LOW);

        blinkArmed = true;
        blinkUntil = millis() + 1000;
        digitalWrite(ARM_LED, armed ? HIGH : LOW);

        AckPacket ack = { MSG_ACK, incomingFireData.id, "IGNITED OK", 1, millis() };
        ack.crc = calculateCRC(ack);
        uint8_t result = esp_now_send(lastSenderMac, (uint8_t *)&ack, sizeof(ack));
        Serial.printf("✅ Structured ACK sent to master, result=%d\n", result);
      }
    } else {
      Serial.println("❌ Invalid FIRE data");
    }
  }

  if (blinkArmed) {
    if (millis() < blinkUntil) {
      digitalWrite(ARM_LED, (millis() / 100) % 2);
    } else {
      blinkArmed = false;
      digitalWrite(ARM_LED, armed ? HIGH : LOW);
    }
  }

  if (blinkRecv) {
    blinkRecv = false;
    blinkLED(LED_RECV);
  }
}
