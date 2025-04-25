#include <esp_now.h>
#include <WiFi.h>

// Define structure for data transmission
typedef struct {
  bool helmetWorn;
  int alcoholLevel;
  int vibrationLevel;
  bool accidentDetected;
} HelmetData;

HelmetData helmetData;

// Pins
#define IR_SENSOR_PIN 34
#define ALCOHOL_SENSOR_PIN 35
#define VIBRATION_SENSOR_PIN 32

// MAC address of the bike unit
uint8_t receiverMAC[] = {0x2C, 0xBC, 0xBB, 0x0D, 0xEC, 0x48};

void setup() {
  Serial.begin(115200);
  Serial.println("🪖 Helmet Unit Starting...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("✅ Peer Added");
  }

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(ALCOHOL_SENSOR_PIN, INPUT);
  pinMode(VIBRATION_SENSOR_PIN, INPUT);
}

void loop() {
  helmetData.helmetWorn = digitalRead(IR_SENSOR_PIN) == LOW;
  helmetData.alcoholLevel = analogRead(ALCOHOL_SENSOR_PIN);
  helmetData.vibrationLevel = analogRead(VIBRATION_SENSOR_PIN);
  helmetData.accidentDetected = helmetData.vibrationLevel > 800;

  // Print sensor values
  Serial.println("\n🔹 Helmet Sensor Readings:");
  Serial.print("🪖 Helmet Worn: "); Serial.println(helmetData.helmetWorn ? "Yes" : "No");
  Serial.print("🍺 Alcohol Level: "); Serial.println(helmetData.alcoholLevel);
  Serial.print("📉 Vibration Level: "); Serial.println(helmetData.vibrationLevel);
  Serial.print("💥 Accident Detected: "); Serial.println(helmetData.accidentDetected ? "Yes" : "No");

  Serial.println("📤 Sending Data...");
  esp_now_send(receiverMAC, (uint8_t*)&helmetData, sizeof(helmetData));

  delay(3000);
}