// Stripped-down test program for ESP-NOW packet reception and SSR activation

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define MAX_SLAVES 5

// Define master MAC globally
uint8_t masterMac[6] = { 0x08, 0x3A, 0x8D, 0xCF, 0xC0, 0x56 }; // Replace with actual master MAC

uint8_t slaveMacs[MAX_SLAVES][6] = {
  { 0x24, 0xD7, 0xEB, 0xCB, 0x47, 0x23 },
  { 0xC8, 0xC9, 0xA3, 0x86, 0x7F, 0x72 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x02 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x03 }
};

#define IS_MASTER false  // Set to false on slave
#define FIRE_PIN D1

void onReceive(uint8_t *mac, uint8_t *data, uint8_t len) {
  Serial.printf("📩 Received packet from %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();

  if (!IS_MASTER && len > 0 && data[0] == 0xF1) {
    Serial.println("🔥 FIRE command received! Activating SSR.");
    digitalWrite(FIRE_PIN, HIGH);
    delay(500);
    digitalWrite(FIRE_PIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  pinMode(FIRE_PIN, OUTPUT);
  digitalWrite(FIRE_PIN, LOW);

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  if (IS_MASTER) {
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    for (int i = 0; i < MAX_SLAVES; i++) {
      esp_now_add_peer(slaveMacs[i], ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    }
    Serial.println("🟢 Master initialized");
  } else {
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    Serial.print("🔵 Slave MAC: ");
    Serial.println(WiFi.macAddress());
  }

  esp_now_register_recv_cb(onReceive);
}

void loop() {
  static unsigned long lastSend = 0;
  if (IS_MASTER && millis() - lastSend > 5000) {
    uint8_t message[] = { 0xF1, 0x01, 0x02, 0x03 };  // 0xF1 = FIRE command
    for (int i = 0; i < MAX_SLAVES; i++) {
      esp_now_send(slaveMacs[i], message, sizeof(message));
    }
    Serial.println("📤 FIRE command sent to slaves");
    lastSend = millis();
  }
} 
