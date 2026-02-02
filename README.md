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

### Add Root CA Cert if Using HiveMQ Private Cloud Broker
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
