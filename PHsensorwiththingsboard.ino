#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DFRobot_ESP_PH.h"
#include "EEPROM.h"

// WiFi Credentials (Update with your WiFi details)
#define WIFI_SSID "jx"         
#define WIFI_PASS "qwer123456789" 

// ThingsBoard Credentials
#define TB_SERVER "thingsboard.cloud"
#define TB_PORT 1883  
#define TOKEN "NCO1iSUbEt2NpA7vDF6O"  // Your ThingsBoard Access Token

// Sensor & ADC Definitions
#define PH_PIN 35
#define ESPADC 4096.0   
#define ESPVOLTAGE 3300 

DFRobot_ESP_PH ph;
float voltage, phValue, temperature = 25;  // Default temperature

WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to WiFi
void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

// Function to connect to ThingsBoard
void connectToThingsBoard() {
  if (!client.connected()) {
    Serial.println("Connecting to ThingsBoard...");

    if (client.connect("ESP32_Client", TOKEN, "")) {
      Serial.println("Connected to ThingsBoard");
    } else {
      Serial.print("Failed to connect. Error code: ");
      Serial.println(client.state());
    }
  }
}

// Function to send pH data to ThingsBoard
void sendDataToThingsBoard(float phValue, float voltage, float temperature) {
  if (!client.connected()) {
    connectToThingsBoard();
  }

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["pH"] = phValue;
  jsonDoc["voltage"] = voltage;
  jsonDoc["temperature"] = temperature;

  // Serialize JSON
  char jsonBuffer[128];
  serializeJson(jsonDoc, jsonBuffer);

  // Publish data
  if (client.publish("v1/devices/me/telemetry", jsonBuffer)) {
    Serial.println("Data sent to ThingsBoard.");
  } else {
    Serial.println("Failed to send data.");
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(32);  // Needed for pH sensor calibration
  ph.begin();
  
  connectToWiFi();
  client.setServer(TB_SERVER, TB_PORT);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  
  if (!client.connected()) {
    connectToThingsBoard();
  }

  client.loop();

  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {  // Every 1 second
    timepoint = millis();

    // Read pH sensor data
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;
    Serial.print("Voltage: ");
    Serial.println(voltage, 4);

    // Simulated temperature value (update with actual sensor if needed)
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println("°C");

    // Get pH value with temperature compensation
    phValue = ph.readPH(voltage, temperature);
    Serial.print("pH: ");
    Serial.println(phValue, 4);

    // Send pH, voltage, and temperature data to ThingsBoard
    sendDataToThingsBoard(phValue, voltage, temperature);
  }

  ph.calibration(voltage, temperature); // Calibration process
}
