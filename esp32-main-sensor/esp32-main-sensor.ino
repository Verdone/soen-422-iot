#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// Microphone Sensor Wiring:
// + pin  -> 3V3
// G pin  -> GND
// AO pin -> GPIO 35 (analog output from sensor)
// DO pin -> GPIO 32 (digital output: HIGH=quiet, LOW=sound detected)

#define MIC_PIN 35      // Microphone analog output (AO)
#define MIC_D0_PIN 32   // Microphone digital output (DO)

// DHT11 Temperature & Humidity Sensor
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Servo Motor (Fan)
#define SERVO_PIN 12
Servo fanServo;

// LED Pins
#define GREEN_LED_PIN 27   // Green LED
#define YELLOW_LED_PIN 14  // Yellow LED
#define RED_LED_PIN 26     // Red LED

// MQTT + WiFi clients
WiFiClient espClient;
PubSubClient client(espClient);

// OLED Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensor identifier and topic
#define LAB_ID "lab-01"              // Used for MQTT topics
#define LAB_DISPLAY_NAME "H-968"      // Used for OLED display
#define COMMAND_TOPIC "lab/commands"

// States
int incomingQueries = 0; // counter for incoming commands
bool fanOn = false;      // fan state
unsigned long lastFanUpdate = 0; // for fan animation timing
int fanPosition = 0;     // current fan servo position (0, 45, or 90)
int occupancyCount = 0;  // current occupancy count from camera

// Thresholds for "temperature fan control" (servo motor)
float highTempThreshold = 24.0; // turn on
float lowTempThreshold = 19.0;  // turn off

// Establishes WiFi Connection
void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// Handle MQTT callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received on topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  incomingQueries++;
  
  // Parse occupancy data from lab-01/occupancy topic
  if (strcmp(topic, "lab-01/occupancy") == 0) {
    // Create null-terminated string from payload
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    
    // Parse "occupancy_count" field from JSON
    char* countPtr = strstr(message, "\"occupancy_count\":");
    if (countPtr != NULL) {
      // Move past the key and whitespace to find the value
      countPtr = strchr(countPtr, ':');
      if (countPtr != NULL) {
        occupancyCount = atoi(countPtr + 1);
        Serial.print("Updated occupancy count: ");
        Serial.println(occupancyCount);
      }
    }
  }
}

// Handle MQTT connection
void connectMQTT() {
  client.setCallback(mqttCallback);
  while (!client.connected()) {
    if (client.connect("ESP32_DHT11_Client")) {
      Serial.println("connected!");
      client.subscribe(COMMAND_TOPIC);
      client.subscribe("lab-01/occupancy");
      Serial.println("Subscribed to lab-01/occupancy");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(", retrying in 3s...");
      delay(3000);
    }
  }
}

// Initial setup
void setup() {
  Serial.begin(115200);
  dht.begin();

  // Microphone digital pin init
  pinMode(MIC_D0_PIN, INPUT_PULLUP);

  // LED pins init
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  // Servo init
  fanServo.attach(SERVO_PIN);
  fanServo.write(0); // neutral/off

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  connectWiFi();
  client.setServer(MQTT_BROKER, MQTT_PORT);
  connectMQTT();
  
  // Set initial status LED (yellow = idle/ready)
  digitalWrite(YELLOW_LED_PIN, HIGH);
}

// Main loop
void loop() {
  if (!client.connected()) {
    // Show red LED when reconnecting
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    connectMQTT();
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, HIGH);
  }
  client.loop();

  // Read sensors
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) { delay(2000); return; }

  // Read analog microphone (AO)
  int micRaw = analogRead(MIC_PIN);
  float micPercent = micRaw / 4095.0 * 100.0;

  // Read digital microphone (DO)
  bool soundDetected = !digitalRead(MIC_D0_PIN); // LOW = sound detected (active LOW)

  const char* micLabel;
  if (micPercent < 30.0) micLabel = "Quiet";
  else if (micPercent < 70.0) micLabel = "Moderate";
  else micLabel = "Loud";

  // Makeshift fan control (aka the servo motor)
  if (!fanOn && t >= highTempThreshold) {
    fanOn = true;
    fanPosition = 0;
    fanServo.write(0);
    lastFanUpdate = millis();
  } else if (fanOn && t <= lowTempThreshold) {
    fanOn = false;
    fanServo.write(0);
  }

  // Animate fan when it's on
  if (fanOn && (millis() - lastFanUpdate >= 1000)) {
    // Cycle through positions: 0 -> 45 -> 90 -> 0
    if (fanPosition == 0) {
      fanPosition = 45;
    } else if (fanPosition == 45) {
      fanPosition = 90;
    } else {
      fanPosition = 0;
    }
    fanServo.write(fanPosition);
    lastFanUpdate = millis();
  }

  // Get current timestamp (millis since boot)
  unsigned long timestamp = millis();
  
  // Publish to dedicated MQTT topics
  char payload[150];
  bool publishSuccess = true;
  
  // Publish temperature
  snprintf(payload, sizeof(payload), "{\"temperature\": %.2f, \"timestamp\": %lu}", t, timestamp);
  publishSuccess &= client.publish("lab-01/temperature", payload);
  
  // Publish humidity
  snprintf(payload, sizeof(payload), "{\"humidity\": %.2f, \"timestamp\": %lu}", h, timestamp);
  publishSuccess &= client.publish("lab-01/humidity", payload);
  
  // Publish noise level
  snprintf(payload, sizeof(payload), "{\"noise_level\": %.1f, \"timestamp\": %lu}", micPercent, timestamp);
  publishSuccess &= client.publish("lab-01/noise-level", payload);
  
  if (publishSuccess) {
    // Success: flash green LED
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(500);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else {
    // Failed: flash red LED
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    delay(500);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, HIGH);
  }

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.printf("Room: %s\n", LAB_DISPLAY_NAME);
  display.printf("Temp: %.2f C\n", t);
  display.printf("Humidity: %.2f %%\n", h);
  display.printf("Sound: %s (%.1f%%)\n", micLabel, micPercent);
  display.printf("Occupancy: %d people\n", occupancyCount);
  display.printf("Fan: %s\n", fanOn ? "ON" : "OFF");

  display.display();

  delay(5000); // update every 5 seconds
}
