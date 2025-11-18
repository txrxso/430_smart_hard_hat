# 430_IMU_GPS_Node

Code for the sensor module (ESP32 + IMU + GPS) that does the following:
- Reads IMU Data from the MPU6050 sensor
- Determines if an impact occurred using IMU Data
- Polls geolocation data using the Neo6M GPS module
- Creates and formats payloads to send to the MQTT broker

TO DO: 
- impact detection code
- CAN code
- FreeRTOS? Potentially using both cores at the same time and different priority of tasks. 



## Add Required Wifi Credentials

1. `touch include/secrets.h`
2. Include the following:

```#ifndef SECRETS_H
#define SECRETS_H

// WPA-2 wifi credentials - change these values to match your CWL
#define WIFI_SSID  "eduroam" 
#define WIFI_EAP_ID "cwl@ubc.ca"
#define WIFI_USER "cwl@ubc.ca"
#define WIFI_PASSWORD "pswd"

// Home wifi credentials or hot spot
#define HOTSPOT_SSID "wifi_ssid" 
#define HOTSPOT_PSWD "wifi_pswd"

#endif 
```
