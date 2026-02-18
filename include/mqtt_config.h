#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

const char* MQTT_SERVER_HIVEMQ_PUBLIC = "broker.hivemq.com"; //"test.mosquitto.org";

const int MQTT_PORT_HIVEMQ_PUBLIC = 1883; 
const char* MQTT_TOPIC_ALERTS = "igen430/shh/alerts/workerA";
const char* MQTT_TOPIC_HEARTBEATS = "igen430/shh/heartbeats/workerA";

// TLS + HIVE MQ PRIVATE BROKER
const char* MQTT_SERVER_HIVEMQ_PRIVATE = "fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud";
const int MQTT_PORT_HIVEMQ_TLS = 8883; 
const char* MQTT_TLS_URL = "fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud:8883";
const char* MQTT_USER = "smarthardhat";
const char* MQTT_PSWD = "Smarthardhat0";

#endif 
