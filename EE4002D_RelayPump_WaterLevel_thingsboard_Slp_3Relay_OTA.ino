#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//OTA
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

// Config Type --> NC: High=Current Flow; NO: High=No Current Flow
#define relay1 5  //NPK --> Plants (Pin 5)
#define relay2 7  //Fish --> NPK (Pin 7)
#define relay3 6  //Clean Water --> Fish (Pin 6)
#define WATER_LEVEL_PIN 1
#define modePin 2

#define WIFI_SSID "Here"     // CHANGE IT
#define WIFI_PASSWORD "passwordishere?"  // CHANGE IT

// Store relay state and water level state
bool relayState1;
bool relayState2;
bool relayState3;
bool currentWLState; // Store the debounced state
bool sleepStatus;

bool WLreading = 0; // When 1, the liquid is detected

const int short_delay = 1000; // Use const for constants
const int long_delay = 5000;
const int timer = 12;   // Sleep wakeup timer (15 for 15 seconds, 43200 for 12 hours)
const float OTAversion = 1.2;

unsigned int Relay1Count = 0;   // Data Msg Counter
unsigned int Relay2Count = 0;
unsigned int Relay3Count = 0;

// ThingsBoard MQTT credentials
const char* mqtt_server = "mqtt.thingsboard.cloud";
const char* telemetry_topic = "v1/devices/me/telemetry";
const char* attributes_topic = "v1/devices/me/attributes";
const char* clientId = "fcd38890-0ae7-11f0-86ac-951bbb28eae1";
const char* access_token = "4qrOh7osl5CUMhhq340u"; 

WiFiClient espClient;
PubSubClient client(espClient);

// OTA
AsyncWebServer server(80);

// ----------------------------------------------------------------------------
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(short_delay);
  }
  Serial.println("\n~~~~~WiFi Connected~~~~~");
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  delay(short_delay);
}

// ----------------------------------------------------------------------------
void initOTA() {
  // For Async ONLY
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = "OTA Version: " + String(OTAversion) + "\n";
    response += "Hi! This is ElegantOTA. Enter update URL by entering IP/update\n";
    response += "Ensure that your new sketch is in .bin file (Sketch > Export Compiled Binary)";

    request->send(200, "text/plain", response);
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
  // After ElegantOTA has started, use and update the IP address in the search bar to 192.169.xx.xx/update
}

// ----------------------------------------------------------------------------
// MQTT Connection, Reconnection, Send
void initMQTT() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard... ");
    if (client.connect(clientId, access_token, NULL)) {
        Serial.println("Connected!");
        client.subscribe("v1/devices/me/rpc/request/+");
    } else {
        Serial.print("Failed, rc=");
        Serial.print(client.state());
        Serial.println(" Trying again in 5 seconds...");
        delay(short_delay);
    }
  }
}

void sendTelemetry(bool currentWLState, int Relay1Count, bool relayState1, bool relayState2, bool relayState3, bool sleepStatus) {
  // JSON document to hold the data
  DynamicJsonDocument doc(1024);
  doc["Sensor Type"] = "Water Level-Relay-Pump";
  doc["Below Water Level"] = currentWLState;
  doc["Reading ID"] = Relay1Count;
  doc["Pump State (From NPK To Plants)"] = relayState1;
  doc["Pump State (From Fish To NPK)"] = relayState2;
  doc["Pump State (From Clean Water To Fish)"] = relayState3;
  doc["Sleep Status"] = sleepStatus;

  // Send JSON data to ThingsBoard
  String payload;
  serializeJson(doc, payload);
  Serial.print("Sending payload: ");
  Serial.println(payload);

  client.publish(telemetry_topic, payload.c_str());

  delay(short_delay);
}

// void callback(char* topic, byte* payload, unsigned int length) {
//   Serial.print("Message arrived on topic: ");
//   Serial.println(topic);
//   callbackFlag = 1;

//   String receivedPayload;
//   for (unsigned int i = 0; i < length; i++) {
//       receivedPayload += (char)payload[i];
//   }
//   Serial.println(receivedPayload);

//   // Decode JSON request
//   DynamicJsonDocument doc(1024);
//   DeserializationError error = deserializeJson(doc, receivedPayload);
//   if (error) {
//       Serial.print("deserializeJson() failed: ");
//       Serial.println(error.c_str());
//       return;
//   }

//   String topicStr = String(topic);
//   int requestIdStart = topicStr.lastIndexOf('/') + 1;
//   String requestId = topicStr.substring(requestIdStart);
//   const char* method = doc["method"];

//   // Relay control
//   if (strcmp(method, "setRelayState") == 0) {
//     bool newRelayState = doc["params"];
//     relayState1 = newRelayState;
//     digitalWrite(relay1, relayState1 ? HIGH : LOW);
//     Serial.println(relayState1 ? "Relay ON (Current Flow)" : "Relay OFF (Current Stop)");

//     sendRelayState();  // Update ThingsBoard with new state

//     String response = "{\"params\":" + String(relayState1 ? "true" : "false") + "}";
//     String responseTopic = "v1/devices/me/rpc/response/" + requestId;
//     client.publish(responseTopic.c_str(), response.c_str());

//     Serial.print("Relay switched to: ");
//     Serial.println(relayState1 ? "ON" : "OFF");
//   }

//   // Get Relay State
//   if (strcmp(method, "getRelayState") == 0) {
//     String response = "{\"relayState1\":" + String(relayState1 ? "true" : "false") + "}";
//     String responseTopic = "v1/devices/me/rpc/response/" + requestId;
//     client.publish(responseTopic.c_str(), response.c_str());
//   }
//   callbackFlag = 0;
// }

// // Function to update Relay state attributes
// void sendRelayState() {
//     String attributeUpdate = "{\"relayState1\":" + String(relayState1 ? "true" : "false") + "}";
//     client.publish(attributes_topic, attributeUpdate.c_str());
// }


// ----------------------------------------------------------------------------
// Water Level Sensor Active Tracking To Control Relay1,2,3
void checkWaterLevelcontrolRelay() {
  if (currentWLState) {
    relayState1 = true;
    digitalWrite(relay1, HIGH);
    Serial.println("Water level HIGH: Relay ON");
  } else {
    relayState1 = false;
    digitalWrite(relay1, LOW);
    Serial.println("Water level LOW: Relay OFF");
    Serial.println();
  }
  // sendRelayState(); // Update ThingsBoard with new state
}

void checkRelay1controlRelay2() {
  Serial.println("~~~~~Relay2 pump START (Fish --> NPK)~~~~~");
  while (Relay2Count != Relay1Count) {
    Relay2Count++;
    relayState2 = true;
    Serial.printf("Relay2 Data Point: %d\n", Relay2Count);
    digitalWrite(relay2, HIGH);
    Serial.println("Relay2 Activated: Relay ON");
    sendTelemetry(currentWLState, Relay1Count, relayState1, relayState2, relayState3, sleepStatus);
    Serial.println("--------------------");
  }
  digitalWrite(relay2, LOW);
  Serial.println("Relay2 Activated: Relay OFF");
  Relay2Count = 0;
  relayState2 = false;
  sendTelemetry(currentWLState, Relay1Count, relayState1, relayState2, relayState3, sleepStatus);
  Serial.println("~~~~~Relay2 pump COMPLETE~~~~~\n");
}

void checkRelay2controlRelay3() {
  Serial.println("~~~~~Relay3 pump START (Clean Water --> Fish)~~~~~");
  while (Relay3Count != Relay1Count) {
    Relay3Count++;
    Serial.printf("Relay3 Data Point: %d\n", Relay3Count);
    relayState3 = true;
    digitalWrite(relay3, HIGH);
    Serial.println("Relay3 Activated: Relay ON");
    sendTelemetry(currentWLState, Relay1Count, relayState1, relayState2, relayState3, sleepStatus);
    Serial.println("--------------------");
  }
  digitalWrite(relay3, LOW);
  Serial.println("Relay3 Activated: Relay OFF");
  Relay3Count = 0;
  relayState3 = false;
  sendTelemetry(currentWLState, Relay1Count, relayState1, relayState2, relayState3, sleepStatus);
  Serial.println("~~~~~Relay3 pump COMPLETE~~~~~\n");
}


// ----------------------------------------------------------------------------
// // Deep Sleep (Exti)
// void deepSleep() {
//   if (currentWLState == false) { // If water level is false (water level is right)
//     Serial.println("Water level normal. Entering Deep Sleep...");
//     esp_sleep_enable_ext0_wakeup((gpio_num_t)WATER_LEVEL_PIN, HIGH);
//     delay(short_delay);

//     esp_deep_sleep_start(); // Enter deep sleep
//   }
//   Serial.println("----------------------------------------------------------------------------");
// }

// // Modem Sleep (may not work)
// void modemSleep() {
//   if (currentWLState == false) { // If water level is false (water level is right)
//     Serial.println("Water level normal. Entering Modem Sleep...");

//     // Disable WiFi to save power
//     WiFi.disconnect(true);
//     WiFi.mode(WIFI_OFF);
//   } else {
//     initWiFi();
//     Serial.println("~~~~~ESP AWAKE: Water needs top up, Relay ON~~~~~");
//   }
// }

// // Light Sleep
// void lightSleep() {
//   if (currentWLState == false) { // If water level is false (water level is right)
//     Serial.println("Water level normal. Entering Light Sleep...");
//     esp_sleep_enable_ext0_wakeup((gpio_num_t)WATER_LEVEL_PIN, HIGH);
//     delay(short_delay);

//     esp_light_sleep_start(); // Enter light sleep (CPU pauses)
//   }
//   Serial.println("----------------------------------------------------------------------------");
// }

// Deep Sleep (Timer)
void deepSleepTimer() {
  Serial.printf("Entering Deep Sleep for %d Hours\n", timer);
  Serial.println("----------------------------------------------------------------------------");
  esp_sleep_enable_timer_wakeup(timer*1000000); // Wake up after 5 seconds (5,000,000us)
  delay(short_delay);

  esp_deep_sleep_start(); // Enter deep sleep
}


void setup() {
  Serial.begin(115200);

  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, LOW); // Keep Water Pump OFF on boot
  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, LOW); // Keep Water Pump OFF on boot
  pinMode(relay3, OUTPUT);
  digitalWrite(relay3, LOW); // Keep Water Pump OFF on boot

  pinMode(WATER_LEVEL_PIN, INPUT);
  pinMode(modePin, OUTPUT);
  digitalWrite(modePin, WLreading);

  initWiFi();
  initOTA();
  Serial.printf("OTA Version: %f\n", OTAversion);

  //thingsbaord server setup
  client.setServer(mqtt_server, 1883);
  // client.setCallback(callback);
}

void loop() {
  initMQTT();

  currentWLState = digitalRead(WATER_LEVEL_PIN); // Important to ensure operation of while()
  Serial.println(currentWLState ? "Below WL State: true" : "Below WL State: false");

  while (currentWLState != false) {
    Relay1Count++;
    Serial.printf("Relay1 Data Point: %d\n", Relay1Count);
    currentWLState = digitalRead(WATER_LEVEL_PIN); // Read the current state
    Serial.println(currentWLState ? "(Loop) Below WL State: true" : "(Loop) Below WL State: false");
    checkWaterLevelcontrolRelay();
    sendTelemetry(currentWLState, Relay1Count, relayState1, relayState2, relayState3, sleepStatus);
    Serial.println("--------------------");

    if (currentWLState == false) {  //If water level is sufficient, activate relay in sequence, then sleep
      checkWaterLevelcontrolRelay();
      delay(short_delay);
      checkRelay1controlRelay2();
      delay(short_delay);
      checkRelay2controlRelay3();

      // Reset Counter
      Relay2Count = 0;
      Relay3Count = 0;
      Serial.printf("Reset Counters: Relay2: %d, Relay3 %d\n", Relay2Count, Relay3Count);

      sleepStatus = true;
      sendTelemetry(currentWLState, Relay1Count, relayState1, relayState2, relayState3, sleepStatus);
      Serial.println("--------------------");
    }
  }

  ElegantOTA.loop();

  delay(short_delay);
  // Uncomment one of  these to test different sleep modes
  // deepSleep();   // Stops everything until external wake-up
  deepSleepTimer();  // Timer wake-up
  // modemSleep();  // WiFi off, CPU stays on
  // lightSleep();  // CPU pauses, RAM & WiFi is on
}