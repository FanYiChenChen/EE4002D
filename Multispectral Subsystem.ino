#include <WiFi.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include "AS726X.h"

/*******************************************************
   Combined ESP32 Program:
   - HC-SR04 Distance Control
   - AS726X Spectral Sensing
   - MQTT Telemetry
*******************************************************/

//================ Ultrasonic & Servo Config ================
#define TRIGGER_PIN 18
#define ECHO_PIN    19
#define MAX_DISTANCE_CM 100
#define TIMEOUT_US (MAX_DISTANCE_CM * 58UL * 2UL)
#define NUM_SAMPLES 5
float targetDistance = 11;
int actuatorPin = 13;
static int actuatorMicros = 1500;
Servo actuator;

//================ AS726X & MQTT Config ====================
const char* ssid = "Huy";
const char* password = "huydinh123";
const char* mqttServer = "mqtt.thingsboard.cloud";
const int mqttPort = 1883;
const char* accessToken = "UG9YAnZsVViioij5C8Xq";
const char* clientId = "b1311bd0-0b20-11f0-86ac-951bbb28eae1";

AS726X sensorAS7262;
AS726X sensorAS7263;
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

#define SATURATION_THRESHOLD 65000
float darkGreen_7262, whiteGreen_7262;
float darkS_7263, darkW_7263;
float whiteS_7263, whiteW_7263;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

//================ Timing Control ====================
unsigned long previousSensorMillis = 0;
const long sensorInterval = 5000;
unsigned long previousUltraMillis = 0;
const long ultraInterval = 2000;

//================ Function Prototypes ================
float readUltrasonicDistanceOnce();
float readUltrasonicDistance();
void connectWiFi();
void connectMQTT();
bool checkSaturation_AS7262();
bool checkSaturation_AS7263();
void darkCal_AS7262();
void whiteCal_AS7262();
void darkCal_AS7263();
void whiteCal_AS7263();
void sendIndicesToThingsBoardMQTT(float ndvi, const String& ndviLabel,
                                  float ndwi, const String& ndwiLabel,
                                  float gci, const String& gciLabel);

//================ Ultrasonic Functions ================
float readUltrasonicDistanceOnce() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
  long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  if (duration == 0) return 0.0;
  return duration / 58.0;
}

float readUltrasonicDistance() {
  float readings[NUM_SAMPLES];
  int validCount = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    float d = readUltrasonicDistanceOnce();
    delay(70);
    readings[i] = d;
    if (d > 0) validCount++;
  }

  if (validCount == 0) return 0.0;

  float sum = 0.0;
  int used = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (readings[i] > 0) {
      sum += readings[i];
      used++;
    }
  }
  return sum / (float)used;
}

//================ AS726X Functions ====================
bool checkSaturation_AS7262() {
  uint16_t rawV = sensorAS7262.getViolet();
  uint16_t rawB = sensorAS7262.getBlue();
  uint16_t rawG = sensorAS7262.getGreen();
  uint16_t rawY = sensorAS7262.getYellow();
  uint16_t rawO = sensorAS7262.getOrange();
  uint16_t rawR = sensorAS7262.getRed();

  bool saturated = false;
  if (rawV >= SATURATION_THRESHOLD) saturated = true;
  if (rawB >= SATURATION_THRESHOLD) saturated = true;
  if (rawG >= SATURATION_THRESHOLD) saturated = true;
  if (rawY >= SATURATION_THRESHOLD) saturated = true;
  if (rawO >= SATURATION_THRESHOLD) saturated = true;
  if (rawR >= SATURATION_THRESHOLD) saturated = true;

  return saturated;
}

bool checkSaturation_AS7263() {
  uint16_t rawR = sensorAS7263.getR();
  uint16_t rawS = sensorAS7263.getS();
  uint16_t rawT = sensorAS7263.getT();
  uint16_t rawU = sensorAS7263.getU();
  uint16_t rawV = sensorAS7263.getV();
  uint16_t rawW = sensorAS7263.getW();

  bool saturated = false;
  if (rawR >= SATURATION_THRESHOLD) saturated = true;
  if (rawS >= SATURATION_THRESHOLD) saturated = true;
  if (rawT >= SATURATION_THRESHOLD) saturated = true;
  if (rawU >= SATURATION_THRESHOLD) saturated = true;
  if (rawV >= SATURATION_THRESHOLD) saturated = true;
  if (rawW >= SATURATION_THRESHOLD) saturated = true;

  return saturated;
}

void darkCal_AS7262() {
  sensorAS7262.disableBulb();
  delay(5000);
  sensorAS7262.takeMeasurements();
  darkGreen_7262 = sensorAS7262.getCalibratedGreen();
}

void whiteCal_AS7262() {
  sensorAS7262.enableBulb();
  delay(5000);
  sensorAS7262.takeMeasurements();
  whiteGreen_7262 = sensorAS7262.getCalibratedGreen();
}

void darkCal_AS7263() {
  sensorAS7263.disableBulb();
  delay(5000);
  sensorAS7263.takeMeasurements();
  darkS_7263 = sensorAS7263.getCalibratedS();
  darkW_7263 = sensorAS7263.getCalibratedW();
}

void whiteCal_AS7263() {
  sensorAS7263.enableBulb();
  delay(5000);
  sensorAS7263.takeMeasurements();
  whiteS_7263 = sensorAS7263.getCalibratedS();
  whiteW_7263 = sensorAS7263.getCalibratedW();
}

//================ MQTT Functions ====================
void sendIndicesToThingsBoardMQTT(float ndvi, const String& ndviLabel,
                                  float ndwi, const String& ndwiLabel,
                                  float gci, const String& gciLabel) {
  if (!mqttClient.connected()) return;

  String payload = "{";
  payload += "\"NDVI\":" + String(ndvi, 3) + ",";
  payload += "\"NDVI_Label\":\"" + ndviLabel + "\",";
  payload += "\"NDWI\":" + String(ndwi, 3) + ",";
  payload += "\"NDWI_Label\":\"" + ndwiLabel + "\",";
  payload += "\"GCI\":" + String(gci, 3) + ",";
  payload += "\"GCI_Label\":\"" + gciLabel + "\"";
  payload += "}";

  mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
}

//================ Core Functions ====================
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void connectMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  while (!mqttClient.connected()) {
    if (mqttClient.connect(clientId, accessToken, NULL)) break;
    delay(3000);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Ultrasonic & Servo Setup
  actuator.attach(actuatorPin, 1000, 2000);
  actuator.writeMicroseconds(1500);
  delay(2000);

  // AS726X & MQTT Setup
  connectWiFi();
  I2C_1.begin(21, 22, 400000);
  I2C_2.begin(25, 26, 400000);
  
  sensorAS7262.begin(I2C_1);
  sensorAS7263.begin(I2C_2);
  
  sensorAS7262.setIntegrationTime(50);
  sensorAS7262.setGain(3);
  sensorAS7263.setIntegrationTime(50);
  sensorAS7263.setGain(3);

  darkCal_AS7262();
  whiteCal_AS7262();
  darkCal_AS7263();
  whiteCal_AS7263();

  connectMQTT();
}

void loop() {
  unsigned long currentMillis = millis();

  // Maintain MQTT connection
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  // Ultrasonic Distance Control
  if (currentMillis - previousUltraMillis >= ultraInterval) {
    previousUltraMillis = currentMillis;
    
    float distance = readUltrasonicDistance();
    Serial.print("Distance: "); Serial.println(distance);
    if (distance == 0) {
      Serial.println("Ultrasonic out of range!");
      return;
    }

    float error = distance - targetDistance;
    int stepSize = 20;

    if (error > 1.5) {
      actuatorMicros -= stepSize;
      if (actuatorMicros < 1000) actuatorMicros = 1000;
    } else if (error < -1.5) {
      actuatorMicros += stepSize;
      if (actuatorMicros > 2000) actuatorMicros = 2000;
    }

    actuator.writeMicroseconds(actuatorMicros);
  }

  // Spectral Sensing & MQTT
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;

    // AS7262 Measurements
    sensorAS7262.takeMeasurements();
    if (checkSaturation_AS7262()) return;
    float rawGreen = sensorAS7262.getCalibratedGreen();
    float reflectGreen = (rawGreen - darkGreen_7262) / (whiteGreen_7262 - darkGreen_7262);

    // AS7263 Measurements
    sensorAS7263.takeMeasurements();
    if (checkSaturation_AS7263()) return;
    float rawS = sensorAS7263.getCalibratedS();
    float rawW = sensorAS7263.getCalibratedW();
    float reflectS = (rawS - darkS_7263) / (whiteS_7263 - darkS_7263);
    float reflectW = (rawW - darkW_7263) / (whiteW_7263 - darkW_7263);

    // Calculate Indices
    float ndvi = (reflectW - reflectS) / (reflectW + reflectS);
    float ndwi = (reflectGreen - reflectW) / (reflectGreen + reflectW);
    float gci = (reflectW / reflectGreen) - 1.0;
    Serial.println("-------------------------------------");
    Serial.print("NDVI: "); Serial.println(ndvi, 3);
    Serial.print("NDWI: "); Serial.println(ndwi, 3);
    Serial.print("GCI : "); Serial.println(gci, 3);
    // Classification
    String ndviLabel, ndwiLabel, gciLabel;
    if (ndvi < 0.0) ndviLabel = "Severely stressed";
    else if (ndvi < 0.2) ndviLabel = "Unhealthy";
    else if (ndvi < 0.6) ndviLabel = "Moderate";
    else ndviLabel = "Healthy";

    if (ndwi < -0.2) ndwiLabel = "Very low water";
    else if (ndwi < 0.0) ndwiLabel = "Low water";
    else if (ndwi < 0.2) ndwiLabel = "Moderate water";
    else ndwiLabel = "High water";

    if (gci < 0.0) gciLabel = "Chl deficiency";
    else if (gci < 0.5) gciLabel = "Low chl";
    else if (gci < 1.5) gciLabel = "Moderate chl";
    else gciLabel = "High chl";
    Serial.println("\nClassification:");
    Serial.print("NDVI => "); Serial.println(ndviLabel);
    Serial.print("NDWI => "); Serial.println(ndwiLabel);
    Serial.print("GCI  => "); Serial.println(gciLabel);
    Serial.println("-------------------------------------\n");
    // Send Data
    sendIndicesToThingsBoardMQTT(ndvi, ndviLabel, ndwi, ndwiLabel, gci, gciLabel);
   
  }
}