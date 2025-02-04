#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <ArduinoJson.h>

// Pin Definitions
#define DHTPIN 4         
#define DHTTYPE DHT22    
#define LIGHT_SENSOR_PIN 19 
#define SOIL_SENSOR_PIN 20  
#define RELAY_PIN 21      
#define NEO_PIN 12        
#define NUMPIXELS 8       

// Soil Moisture Thresholds
#define SOIL_MOISTURE_DRY_MAX 2732    
#define SOIL_MOISTURE_DRY_MIN 2699    
#define SOIL_MOISTURE_WET 1800        

// Update these constants at the top - remove the old LED scaling defines and replace with these
#define LIGHT_MIN 0
#define LIGHT_MAX 4095
#define BRIGHTNESS_MIN 5      // Minimum LED brightness
#define BRIGHTNESS_MAX 255    // Maximum LED brightness
#define DARK_VALUE 3000      // MH sensor reads HIGH in darkness
#define BRIGHT_VALUE 500     // MH sensor reads LOW in bright light

// Initialize sensors and LED
DHT dht(DHTPIN, DHTTYPE);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

// WiFi credentials
const char* ssid = "bingus_soup"; 
const char* password = "12345678"; 

// ThingsBoard MQTT settings
const char* mqtt_server = "mqtt.thingsboard.cloud"; 
const char* access_token = "ukyqz7yul75krrku8i1u"; 
const char* telemetry_topic = "v1/devices/me/telemetry";
const char* attributes_topic = "v1/devices/me/attributes";
const char* clientId = "ESP32"; 

WiFiClient espClient;
PubSubClient client(espClient);

// Global variables for LED control and mode management
String mode = "auto";  
bool isLEDTestActive = false;
int manualBrightness = 0;
int lastBrightness = 0;
bool pumpState = false;

// Timing variables
unsigned long lastUpdateTime = 0;
unsigned long lastTelemetrySentTime = 0;
unsigned long lastDHTReadTime = 0;
const unsigned long manualModeInterval = 30000;  // 30 seconds timeout
const unsigned long telemetryInterval = 10000;   // 10 seconds between telemetry
const unsigned long dhtReadInterval = 2000;      // 2 seconds between DHT readings

// Sensor reading variables
float lastTemperature = 0;
float lastHumidity = 0;
bool dhtReadSuccess = false;

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);  
    dht.begin();
    pinMode(RELAY_PIN, OUTPUT);
    strip.begin();
    strip.show();
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// Update reconnect function to ensure correct initial state
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(clientId, access_token, NULL)) {
            Serial.println("connected");
            client.subscribe("v1/devices/me/rpc/request/+");
            
            // Send current state after connection
            String initialState = "{\"mode\":\"" + mode + "\",\"LEDtest\":" + 
                                String(isLEDTestActive ? "true" : "false") + "}";
            client.publish(attributes_topic, initialState.c_str());
            
            // Debug print current state
            Serial.println("Connected with state:");
            Serial.print("  Mode: ");
            Serial.println(mode);
            Serial.print("  isLEDTestActive: ");
            Serial.println(isLEDTestActive);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setStripBrightness(int brightness, bool isManual = false) {
    brightness = constrain(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    
    if (isManual) {
        manualBrightness = brightness;
        // Only update lastBrightness in manual mode if we're actually in manual mode
        if (mode == "manual") {
            lastBrightness = brightness;
        }
    } else {
        // Only update lastBrightness in auto mode if we're actually in auto mode
        if (mode == "auto") {
            lastBrightness = brightness;
        }
    }
    
    // Actually set the LED brightness
    for (int i = 0; i < NUMPIXELS; i++) {
        strip.setPixelColor(i, strip.Color(brightness, brightness, brightness)); 
    }
    strip.show();
    
    Serial.print("LED Brightness set to: ");
    Serial.print(brightness);
    Serial.print(" (Mode: ");
    Serial.print(mode);
    Serial.print(", isManual: ");
    Serial.print(isManual);
    Serial.println(")");
}

// Update switchToAutoMode function
void switchToAutoMode() {
    mode = "auto";
    isLEDTestActive = false;
    Serial.println("Switching to auto mode");
    
    // Calculate and apply auto brightness
    int lightValue = analogRead(LIGHT_SENSOR_PIN);
    int brightness = logBrightnessMapping(lightValue);
    setStripBrightness(brightness, false);
    
    // Update shared attributes
    String attributeUpdate = "{\"mode\":\"auto\",\"LEDtest\":false}";
    client.publish(attributes_topic, attributeUpdate.c_str());

    // Debug print
    Serial.println("Auto mode activated with settings:");
    Serial.print("  Light value: ");
    Serial.println(lightValue);
    Serial.print("  Calculated brightness: ");
    Serial.println(brightness);
    Serial.print("  isLEDTestActive: ");
    Serial.println(isLEDTestActive);
}


// Update switchToManualMode function
void switchToManualMode() {
    mode = "manual";
    isLEDTestActive = true;
    lastUpdateTime = millis();
    Serial.println("Switching to manual mode");
    
    // Keep current manual brightness or set to a default if none
    if (manualBrightness == 0) {
        manualBrightness = 50;
    }
    setStripBrightness(manualBrightness, true);
    
    // Update shared attributes
    String attributeUpdate = "{\"mode\":\"manual\",\"LEDtest\":true}";
    client.publish(attributes_topic, attributeUpdate.c_str());

    // Debug print
    Serial.println("Manual mode activated with settings:");
    Serial.print("  Brightness: ");
    Serial.println(manualBrightness);
    Serial.print("  isLEDTestActive: ");
    Serial.println(isLEDTestActive);
}

int normalizeSoilMoisture(int rawValue) {
    const int SOIL_MOISTURE_DRY = (SOIL_MOISTURE_DRY_MAX + SOIL_MOISTURE_DRY_MIN) / 2;
    
    if (rawValue > SOIL_MOISTURE_DRY_MAX) {
        rawValue = SOIL_MOISTURE_DRY_MAX;
    }
    
    rawValue = constrain(rawValue, SOIL_MOISTURE_WET, SOIL_MOISTURE_DRY_MAX);
    int normalizedValue = map(rawValue, SOIL_MOISTURE_DRY_MAX, SOIL_MOISTURE_WET, 0, 100);
    return constrain(normalizedValue, 0, 100);
}

// Updated brightness mapping function for MH sensor
int logBrightnessMapping(int lightValue) {
    // Ensure the light value is within bounds
    lightValue = constrain(lightValue, LIGHT_MIN, LIGHT_MAX);
    
    // In dark conditions (HIGH reading), LED should be brighter
    if (lightValue > DARK_VALUE) {
        return BRIGHTNESS_MAX;  // Maximum brightness when dark
    }
    
    // In bright conditions (LOW reading), LED should be dimmer
    if (lightValue < BRIGHT_VALUE) {
        return BRIGHTNESS_MIN;  // Minimum brightness when bright
    }
    
    // Direct mapping: higher sensor value = higher LED brightness
    float brightness = map(lightValue, BRIGHT_VALUE, DARK_VALUE, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
    return (int)brightness;
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long currentTime = millis();

    // Check for manual mode timeout only if no LED test is active
    if (mode == "manual" && 
        (currentTime - lastUpdateTime >= manualModeInterval) && 
        !isLEDTestActive) {
        // Add debug print before switching
        Serial.println("Manual mode timeout - switching to auto");
        switchToAutoMode();
    }

    // Handle telemetry updates
    if (currentTime - lastTelemetrySentTime >= telemetryInterval) {
        lastTelemetrySentTime = currentTime;

        // DHT sensor readings - do this in both modes
        if (currentTime - lastDHTReadTime >= dhtReadInterval) {
            lastDHTReadTime = currentTime;
            float newHumidity = dht.readHumidity();
            float newTemperature = dht.readTemperature();
            
            if (!isnan(newHumidity) && !isnan(newTemperature)) {
                lastHumidity = newHumidity;
                lastTemperature = newTemperature;
                dhtReadSuccess = true;
            } else {
                dhtReadSuccess = false;
                Serial.println("Failed to read from DHT22!");
            }
        }

        // Read all sensors regardless of mode
        int lightValue = analogRead(LIGHT_SENSOR_PIN);
        int rawSoilMoisture = analogRead(SOIL_SENSOR_PIN);
        int soilMoisturePercent = normalizeSoilMoisture(rawSoilMoisture);

        // Update the auto mode section in loop():
        // LED control - ONLY in auto mode
        if (mode == "auto") {
            int brightness = logBrightnessMapping(lightValue);
            setStripBrightness(brightness, false);
        }
        // In manual mode, the LED brightness remains at manualBrightness
        // which is controlled by the MQTT callbacks

        // Automated pump control - works the same in both modes
        if (soilMoisturePercent < 20) {
            digitalWrite(RELAY_PIN, HIGH);
            pumpState = true;
        } else if(soilMoisturePercent > 40) {
            digitalWrite(RELAY_PIN, LOW);
            pumpState = false;
        }

         String payload = "{";
        if (dhtReadSuccess) {
            payload += "\"temperature\":" + String(lastTemperature) + ",";
            payload += "\"humidity\":" + String(lastHumidity) + ",";
        }
        payload += "\"light\":" + String(lightValue) + ",";
        payload += "\"soilMoisture\":" + String(soilMoisturePercent) + ",";
        payload += "\"pumpState\":" + String(pumpState ? "true" : "false") + ",";
        payload += "\"mode\":\"" + mode + "\",";
        
        // Always report the mode-appropriate brightness
        int reportedBrightness = (mode == "manual") ? manualBrightness : lastBrightness;
        payload += "\"brightness\":" + String(reportedBrightness);
        payload += "}";

        client.publish(telemetry_topic, payload.c_str());
    }

    delay(100); // Short delay for stability
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    
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

    // Handle LED mode switching - check for both possible method names
    if (strcmp(method, "setStateLEDtest") == 0 || strcmp(method, "setValueLED") == 0) {
        bool newMode = doc["params"];
        
        Serial.print("Received mode change request: ");
        Serial.println(newMode ? "manual" : "auto");

        if (newMode) {
            mode = "manual";
            isLEDTestActive = true;
            switchToManualMode();
        } else {
            mode = "auto";
            isLEDTestActive = false;
            switchToAutoMode();
        }

        // Send response back to ThingsBoard
        String response = "{\"params\":" + String(newMode ? "true" : "false") + "}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());
        
        // Update shared attributes
        String attributeUpdate = "{\"mode\":\"" + mode + "\",\"LEDtest\":" + 
                               String(isLEDTestActive ? "true" : "false") + "}";
        client.publish(attributes_topic, attributeUpdate.c_str());

        Serial.print("Mode switched to: ");
        Serial.println(mode);
        Serial.print("isLEDTestActive: ");
        Serial.println(isLEDTestActive);
    }


      // Handle LED brightness control - check for both possible method names
    if (strcmp(method, "setValueLEDbright") == 0 || strcmp(method, "setValuebrightness") == 0) {
        if (mode == "manual") {
            int brightness = doc["params"];
            setStripBrightness(brightness, true);
            lastUpdateTime = millis();
            
            String response = "{\"params\":" + String(brightness) + "}";
            String responseTopic = "v1/devices/me/rpc/response/" + requestId;
            client.publish(responseTopic.c_str(), response.c_str());
            
            Serial.print("Manual brightness set to: ");
            Serial.println(brightness);
        } else {
            String response = "{\"params\":" + String(lastBrightness) + "}";
            String responseTopic = "v1/devices/me/rpc/response/" + requestId;
            client.publish(responseTopic.c_str(), response.c_str());
            
            Serial.println("Ignoring manual brightness change in auto mode");
        }
    }

    // Handle pump control
    if (strcmp(method, "setStatePump") == 0) {
        bool newPumpState = doc["params"];
        pumpState = newPumpState;
        digitalWrite(RELAY_PIN, pumpState ? HIGH : LOW);
        
        String response = "{\"params\":" + String(pumpState ? "true" : "false") + "}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());
        
        String attributeUpdate = "{\"pumpState\":" + String(pumpState ? "true" : "false") + "}";
        client.publish(attributes_topic, attributeUpdate.c_str());
    }

    // Get current states
    if (strcmp(method, "getStateLEDtest") == 0) {
        String response = "{\"mode\":\"" + mode + "\"}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());
    }

    if (strcmp(method, "getValueLEDbright") == 0) {
        String response = "{\"brightness\":" + String(lastBrightness) + "}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());
    }

    if (strcmp(method, "getStatePump") == 0) {
        String response = "{\"pumpState\":" + String(pumpState ? "true" : "false") + "}";
        String responseTopic = "v1/devices/me/rpc/response/" + requestId;
        client.publish(responseTopic.c_str(), response.c_str());
    }
}
