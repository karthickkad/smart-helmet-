#include <WiFi.h>
#include <esp_now.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Structure to receive data
typedef struct {
  bool helmetWorn;
  int alcoholLevel;
  int vibrationLevel;
  bool accidentDetected;
} HelmetData;

HelmetData receivedData;

// GPS & GSM
#define RXPin 16  // GPS TX → ESP32 RX
#define TXPin 17  // GPS RX → ESP32 TX
#define SIM_RX 26 // GSM RX → ESP32 TX
#define SIM_TX 27 // GSM TX → ESP32 RX
HardwareSerial gpsSerial(1);
HardwareSerial sim800l(2);
TinyGPSPlus gps;

// Driver Module (L298N)
#define IN1_PIN 13    // Motor direction pin 1
#define IN2_PIN 14    // Motor direction pin 2

// LED Indicators
#define GREEN_LED_PIN 2
#define RED_LED_PIN 15

// Alcohol Threshold
#define ALCOHOL_THRESHOLD 1800

// Blinking LED timer
unsigned long lastBlink = 0;
bool ledState = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
  sim800l.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("\n⏳ Testing Driver Module...");
  controlDriverModule(true);
  delay(2000);
  controlDriverModule(false);
  Serial.println("✅ Driver Module Test Complete");
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  Serial.println("\n📶 Data Received from Helmet Unit:");
  Serial.print("🪖 Helmet Worn: ");
  Serial.println(receivedData.helmetWorn ? "Yes" : "No");
  Serial.print("🍺 Alcohol Level: ");
  Serial.println(receivedData.alcoholLevel);
  Serial.print("📳 Vibration: ");
  Serial.println(receivedData.vibrationLevel);
  Serial.print("💥 Accident: ");
  Serial.println(receivedData.accidentDetected ? "Yes" : "No");

  float latitude = 0.0, longitude = 0.0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.location.isUpdated()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        break;
      }
    }
    if (gps.location.isUpdated()) break;
  }

  Serial.print("📍 Location: ");
  Serial.print(latitude, 6);
  Serial.print(", ");
  Serial.println(longitude, 6);

  if (receivedData.accidentDetected) {
    sendEmergencySMS(latitude, longitude);
  }

  if (receivedData.helmetWorn && receivedData.alcoholLevel < ALCOHOL_THRESHOLD && !receivedData.accidentDetected) {
    controlDriverModule(true);
    digitalWrite(RED_LED_PIN, LOW);
    Serial.println("✅ Safe: Driver ON → LED ON");
  } else {
    controlDriverModule(false);
    digitalWrite(RED_LED_PIN, HIGH);
    Serial.println("⚠️ Unsafe: Driver OFF → LED OFF");
  }

  Serial.println("---------------------------------");
}

void controlDriverModule(bool state) {
  if (state) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  digitalWrite(GREEN_LED_PIN, state);
}

void sendEmergencySMS(float lat, float lon) {
  int signalStrength = checkSignalStrength();
  Serial.print("📶 GSM Signal Strength: ");
  if (signalStrength >= 0) {
    Serial.print(signalStrength);
    Serial.println(" dBm");
  } else {
    Serial.println("❌ Could not retrieve signal strength.");
  }

  sim800l.println("AT+CMGF=1");
  delay(200);
  sim800l.println("AT+CMGS=\"+911234567890\"");
  delay(200);

  String message = "🚨 Emergency!\n";
  message += "Helmet Worn: " + String(receivedData.helmetWorn ? "Yes" : "No") + "\n";
  message += "Alcohol: " + String(receivedData.alcoholLevel) + "\n";
  message += "Accident Detected!\n";
  message += "Location: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6);

  sim800l.print(message);
  sim800l.write(26);
  delay(1000);
}

int checkSignalStrength() {
  sim800l.println("AT+CSQ");
  delay(200);
  String response = "";
  unsigned long startTime = millis();

  while (millis() - startTime < 1000) {
    while (sim800l.available()) {
      char c = sim800l.read();
      response += c;
    }
  }

  int rssiIndex = response.indexOf("+CSQ:");
  if (rssiIndex != -1) {
    int commaIndex = response.indexOf(",", rssiIndex);
    if (commaIndex != -1) {
      String rssiStr = response.substring(rssiIndex + 6, commaIndex);
      int rssi = rssiStr.toInt();
      if (rssi == 99) return -1;
      return -113 + (rssi * 2);
    }
  }
  return -1;
}

void loop() {
  bool driverActive = digitalRead(IN1_PIN) && !digitalRead(IN2_PIN);
  digitalWrite(GREEN_LED_PIN, driverActive);

  if (!driverActive && millis() - lastBlink >= 1000) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(RED_LED_PIN, ledState);
  }
}
