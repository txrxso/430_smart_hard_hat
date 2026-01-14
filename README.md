# 430_gateway_node

## Parameters to change for compilation
In `main.cpp`: 
1. Change DEBUG_MODE to 0 to compile without extra `Serial.println()` debug statements.
2. Change `HEARTBEAT_INTERVAL_MIN` to the desired value in minutes. 

```
// define DEBUG mode to print stuff
#define DEBUG_MODE 1 
#define HEARTBEAT_INTERVAL_MIN 5 // in minutes
```

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
## To run native tests
Ensure that:
1) `gcc --version` returns some version from CMD. Otherwise, follow: https://code.visualstudio.com/docs/cpp/config-mingw to add MinGW-w64 `bin` to the `PATH` environment variable. 
