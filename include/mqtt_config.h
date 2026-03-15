#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define MQTT_SERVER_HIVEMQ_PUBLIC "broker.hivemq.com" //"test.mosquitto.org"

#define MQTT_PORT_HIVEMQ_PUBLIC 1883 
#define MQTT_TOPIC_ALERTS "igen430/shh/alerts/workerA"
#define MQTT_TOPIC_HEARTBEATS "igen430/shh/heartbeats/workerA"

// TLS + HIVE MQ PRIVATE BROKER
#define MQTT_SERVER_HIVEMQ_PRIVATE "fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud"
#define MQTT_PORT_HIVEMQ_TLS 8883 
#define MQTT_TLS_URL "fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud:8883"
#define MQTT_USER "smarthardhat"
#define MQTT_PSWD "Smarthardhat0"

#endif 
