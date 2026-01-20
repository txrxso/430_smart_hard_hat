# 430_gateway_node

## Debug modes 
Change the following flags in `include/sensors.h` : 

```c++
#define IMU_DEBUG 1 // turned on
#define GPS_DEBUG 0 // turned off
#define MQTT_DEBUG 1 
```

Change the following flags in `include/can.h` : 

```c++
#define CAN_DEBUG 1
#define HEARTBEAT_DEBUG 1
```



## Add Required Wifi Credentials

1. `touch include/secrets.h`
2. Include the following:

```c++
#ifndef SECRETS_H
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
