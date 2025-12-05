#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);


// Camera pin definition for AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// connect to MQTT message broker
void connectMQTT() {
  Serial.print("Connecting to MQTT broker...");
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    String clientId = "ESP32-CAM-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" connected!");
      Serial.print("Client ID: ");
      Serial.println(clientId);
      return;
    } else {
      Serial.print(".");
      Serial.print(" (rc=");
      Serial.print(mqttClient.state());
      Serial.print(")");
      attempts++;
      delay(2000);
    }
  }
  if (!mqttClient.connected()) {
    Serial.println(" FAILED!");
    Serial.println("Check:");
    Serial.println("  - Mosquitto broker is running");
    Serial.println("  - Broker IP address is correct");
    Serial.println("  - Port 1883 is accessible");
  }
}

// initial setup
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Starting...");

  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Image quality settings
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // 1600x1200
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera initialized");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Setup and connect to MQTT broker
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(CHUNK_SIZE + 512);  // Buffer for chunks + overhead
  mqttClient.setKeepAlive(60);
  
  connectMQTT();
  
  Serial.println("\n--- Ready to send images via MQTT ---\n");
  delay(2000);
}

// main loop
void loop() {
  // Reconnect MQTT if needed
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected. Reconnecting...");
    connectMQTT();
    if (!mqttClient.connected()) {
      Serial.println("Failed to reconnect. Waiting 10s...");
      delay(10000);
      return;
    }
  }
  mqttClient.loop();
  
  // Take picture
  Serial.println("Taking picture...");
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(5000);
    return;
  }
  Serial.printf("Picture taken! Size: %d bytes\n", fb->len);

  // Send image via MQTT
  if(WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    Serial.print("Publishing image to MQTT topic: ");
    Serial.println(mqtt_topic);
    Serial.printf("Image size: %d bytes\n", fb->len);
    
    // Generate unique message ID for this image
    String msgId = String(millis());
    
    // Calculate number of chunks needed
    int totalChunks = (fb->len + CHUNK_SIZE - 1) / CHUNK_SIZE;
    Serial.printf("Splitting into %d chunks of %d bytes\n", totalChunks, CHUNK_SIZE);
    
    // First, send metadata about the incoming image
    char metadata[256];
    snprintf(metadata, sizeof(metadata),
             "{\"device_id\":\"%s\",\"msg_id\":\"%s\",\"total_size\":%d,\"total_chunks\":%d,\"chunk_size\":%d}",
             device_id, msgId.c_str(), fb->len, totalChunks, CHUNK_SIZE);
    
    if(!mqttClient.publish(mqtt_topic_metadata, metadata)) {
      Serial.println("Failed to publish metadata");
      esp_camera_fb_return(fb);
      delay(30000);
      return;
    }
    Serial.println("Metadata published");
    
    // Send image in chunks
    bool allChunksSuccess = true;
    for(int chunk = 0; chunk < totalChunks; chunk++) {
      int offset = chunk * CHUNK_SIZE;
      int chunkLen = min(CHUNK_SIZE, (int)(fb->len - offset));
      
      // Create topic for this chunk: esp32/camera/MSG_ID/CHUNK_NUM
      String chunkTopic = String(mqtt_topic) + "/" + msgId + "/" + String(chunk);
      
      Serial.printf("Publishing chunk %d/%d (%d bytes)...", chunk + 1, totalChunks, chunkLen);
      
      bool published = mqttClient.publish(chunkTopic.c_str(), fb->buf + offset, chunkLen, false);
      
      if(published) {
        Serial.println("");
      } else {
        Serial.println("FAILED");
        Serial.printf("  Error state: %d\n", mqttClient.state());
        allChunksSuccess = false;
        break;
      }
      
      // Small delay between chunks to avoid overwhelming the broker
      delay(10);
      mqttClient.loop();
    }
    
    if(allChunksSuccess) {
      Serial.println("All chunks published successfully!");
      Serial.printf("Message ID: %s\n", msgId.c_str());
    } else {
      Serial.println("Failed to publish complete image");
      Serial.println("Some chunks failed to send");
    }
  } else {
    Serial.println("WiFi or MQTT not connected!");
  }

  // Release camera frame buffer
  esp_camera_fb_return(fb);

  // Wait before taking next picture (30 seconds)
  Serial.println("Waiting 30 seconds...\n");
  delay(30000);
}