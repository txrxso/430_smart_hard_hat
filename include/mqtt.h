#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include "mqtt_config.h"

#define ENABLE_TLS 1
#define MAX_IMMEDIATE_MQTT_RETRIES 3
#define MQTT_RETRY_DELAY_MS 100
#define DUTY_CYCLE 0 // set to 1 to enable tx_duration measurements for duty cycle calculation

// declare mqttClient (global in main.cpp) as extern 
extern PubSubClient mqttClient;

bool connectToMQTT(bool enableTLS = ENABLE_TLS);

#endif