# Smart Hard Hat Gateway Firmware
Firmware for gateway module ESP32 responsible for the following on the smart hard hat: 
- telemetry via MQTT (heartbeats, alert events)
- CAN communication (RTR for heartbeats and receiving alerts from other nodes on the bus such as noise and air quality)
- sampling of the IMU sensor to continuously monitor for falls/impacts
- sampling to of the GPS module to continuously update geolocation coordinates and sync datetime

# Installation & Usage
```
# Clone the repository
git clone https://github.com/txrxso/430_smart_hard_hat.git
```
##  Configuration
## Debug modes 
Change the following flags in `include/debug.h` : 

```c++
#define IMU_DEBUG 0 // 2 for more detailed IMU printing
#define GPS_DEBUG 1
#define MQTT_DEBUG 1 
#define CAN_DEBUG 1 // 2 to also print when no message received within timeout window, which can be useful for monitoring heartbeat timeouts, etc.
#define HEARTBEAT_DEBUG 1

// flags for mocks
#define MOCK_GPS 0 // 0 = do not use mocks
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

## Add Root CA Cert if Using HiveMQ Private Cloud Broker
To connect to `fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud` on port `8883` (TLS only), we need to set the Root CA certificate.

1. In bash, run `touch include/certs.h`.
2. Include the following: 
```c++
#ifndef CERTS_H
#define CERTS_H

// HiveMQ Cloud Root CA Certificate
const char* root_ca = <PASTE CERTIFICATE HERE>;

// the certificate that should be pasted includes the line 'BEGIN CERTIFICATE' up to, and including the line 'END CERTIFICATE'

#endif 
```
For how to get the certificate for the broker on HIVE MQ cloud, refer to `docs/certificates`.

3.  In `main.cpp`, set `#define ENABLE_TLS 1`.

## Usage 
1. Connect the ESP32 to the peripherals.
2. Connect ESP32 to host with cloned repository. Ensure that the host can detect the right COM port to the ESP32. 
3. With PlatformIO extension in VS code, run `pio run --target upload`.