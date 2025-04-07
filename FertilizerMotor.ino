#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Pin Definitions
#define MOTOR_PIN 4  // GPIO connected to MOSFET signal

// WiFi credentials
const char* ssid = "Chloeee"; 
const char* password = "12345678"; 

// ThingsBoard MQTT settings
const char* mqtt_server = "mqtt.thingsboard.cloud"; 
const char* access_token = "ER1tB8orbhbdwVxtbAFt"; 
const char* telemetry_topic = "v1/devices/me/telemetry";
const char* attributes_topic = "v1/devices/me/attributes";
const char* clientId = "8c393980-eb7b-11ef-91e7-4fe2c52e54e3"; 

WiFiClient espClient;
PubSubClient client(espClient);

bool motorState = false;  // Stores the motor state (ON/OFF)
unsigned long lastTelemetrySentTime = 0; 
const unsigned long telemetryInterval = 3000;  // Send telemetry every 3 seconds

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW); // Ensure motor is OFF initially
}

void setup_wifi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempt++;
        if (attempt > 20) {  // Reset if connection fails after 10s
            Serial.println("\nWiFi connection failed, restarting...");
            ESP.restart();
        }
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(clientId, access_token, NULL)) {
            Serial.println("Connected");
            client.subscribe("v1/devices/me/rpc/request/+");

            // Send initial motor state
            sendMotorState();
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long currentTime = millis();
    if (currentTime - lastTelemetrySentTime >= telemetryInterval) {
        lastTelemetrySentTime = currentTime;
        sendTelemetry();
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message received [");
    Serial.print(topic);
    Serial.print("]: ");

    String receivedPayload;
    for (unsigned int i = 0; i < length; i++) {
        receivedPayload += (char)payload[i];
    }
    Serial.println(receivedPayload);

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, receivedPayload);
    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    String topicStr = String(topic);
    int requestIdStart = topicStr.lastIndexOf('/') + 1;
    String requestId = topicStr.substring(requestIdStart);
    const char* method = doc["method"];

    // Motor control
    if (strcmp(method, "setMotorState") == 0) {
        bool newMotorState = doc["params"];
        motorState = newMotorState;
        digitalWrite(MOTOR_PIN, motorState ? HIGH : LOW);

        sendMotorState();  // Update ThingsBoard with new state

        String response = "{\"params\":" + String(motorState ? "true" : "false") + "}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());

        Serial.print("Motor switched to: ");
        Serial.println(motorState ? "ON" : "OFF");
    }

    // Get motor state
    if (strcmp(method, "getMotorState") == 0) {
        String response = "{\"motorState\":" + String(motorState ? "true" : "false") + "}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());
    }
}

// Function to send motor state telemetry
void sendTelemetry() {
    String payload = "{\"motorState\":" + String(motorState ? "true" : "false") + "}";
    client.publish(telemetry_topic, payload.c_str());
}

// Function to update motor state attributes
void sendMotorState() {
    String attributeUpdate = "{\"motorState\":" + String(motorState ? "true" : "false") + "}";
    client.publish(attributes_topic, attributeUpdate.c_str());
}
