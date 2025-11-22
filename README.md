# soen-422-iot
Embedded system component for an IoT prototype that monitors lab occupancy and noise conditions.

# Development

Your Arduino IDE setup should follow the same configuration (upload speed, partition scheme) from previous SOEN 422 labs. You can also modify the existing sensor/pin settings to your configuration.

In the sketch folder, add your own config file that follows the following format:

```
#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
#define WIFI_SSID       ""
#define WIFI_PASSWORD   ""

// Local Mosquitto broker
#define MQTT_BROKER   ""  //
#define MQTT_PORT     1883
#define MQTT_USER     ""  // optional, can be left blank.
#define MQTT_PASS     ""  // optional, can be left blank.

#endif
```