#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include "config.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// Sensor/pin settings
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#define MIC_PIN 34
#define SERVO_PIN 12
Servo fanServo;

// MQTT + WiFi clients
WiFiClient espClient;
PubSubClient client(espClient);

// OLED Setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensor identifier and topic
#define LAB_ID "lab H-917"
#define COMMAND_TOPIC "lab/commands"

// States
int incomingQueries = 0; // counter for incoming commands
bool fanOn = false;      // fan state

// Thresholds for "temperature fan control" (servo motor)
float highTempThreshold = 30.0; // turn on
float lowTempThreshold = 28.0;  // turn off

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
  Serial.print("Received command: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  incomingQueries++;
}

// Handle MQTT connection
void connectMQTT() {
  client.setCallback(mqttCallback);
  while (!client.connected()) {
    if (client.connect("ESP32_DHT11_Client")) {
      Serial.println("connected!");
      client.subscribe(COMMAND_TOPIC);
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
}

// Main loop
void loop() {
  if (!client.connected()) connectMQTT();
  client.loop();

  // Read sensors
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) { delay(2000); return; }

  int micRaw = analogRead(MIC_PIN);
  float micPercent = micRaw / 4095.0 * 100.0;

  const char* micLabel;
  if (micPercent < 30.0) micLabel = "Quiet";
  else if (micPercent < 70.0) micLabel = "Moderate";
  else micLabel = "Loud";

  // Makeshift fan control (aka the servo motor)
  if (!fanOn && t >= highTempThreshold) {
    fanServo.write(90); // turn on
    fanOn = true;
  } else if (fanOn && t <= lowTempThreshold) {
    fanServo.write(0);  // turn off
    fanOn = false;
  }

  // Publish MQTT
  char payload[150];
  snprintf(payload, sizeof(payload),
           "{\"id\": \"%s\", \"temperature\": %.2f, \"humidity\": %.2f, \"sound\": %.1f}",
           LAB_ID, t, h, micPercent);
  client.publish("test/soen422", payload);

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.printf("Room no.: %s\n", LAB_ID);
  display.printf("Temp: %.2f C\n", t);
  display.printf("Humidity: %.2f %%\n", h);
  display.printf("Sound: %s (%.1f%%)\n", micLabel, micPercent);
  display.printf("Incoming Queries: %d\n", incomingQueries);
  display.printf("Fan: %s\n", fanOn ? "ON" : "OFF");

  display.display();

  delay(5000); // update every 5 seconds
}
