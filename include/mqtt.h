#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include "mqtt_config.h"

#define ENABLE_TLS 1
#define MAX_IMMEDIATE_MQTT_RETRIES 3
#define MQTT_RETRY_DELAY_MS 100

// declare mqttClient (global in main.cpp) as extern 
extern PubSubClient mqttClient;

bool connectToMQTT(bool enableTLS = ENABLE_TLS);

#endif