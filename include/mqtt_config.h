#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

const char* MQTT_SERVER_PUBLIC_MOSQUITTO = "broker.hivemq.com"; //"test.mosquitto.org";
const char* MQTT_SERVER_HIVEMQ = "fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud";

 //"test.mosquitto.org";  - works
 //"10.0.0.115"; - home wifi with local broker running on computer
 //"206.87.235.56" - will not work - eduroam seems to block p2p? 
 //"fe26426fbe64463790fc2792777c8189.s1.eu.hivemq.cloud" - Hive MQ cluster 
const char* MQTT_USER = "smarthardhat";
const char* MQTT_PSWD = "Smarthardhat0";
const int MQTT_PORT_HIVEMQ = 8883; 
const int MQTT_PORT_MOSQUITTO = 1883; 
const char* MQTT_TOPIC_TEST = "20251122_YVR/workerA/mpu6050/";
const char* MQTT_TOPIC_ALERTS = "igen430/shh/alerts/workerA";
const char* MQTT_TOPIC_HEARTBEATS = "igen430/shh/heartbeats/workerA";
const unsigned long PUBLISH_INTERVAL = 500;
const unsigned long RECONNECT_INTERVAL = 2000;

#endif 
