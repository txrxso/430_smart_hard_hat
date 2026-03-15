#include "mqtt.h"
#include "debug.h"
#include "wifi_config.h"
#include <Arduino.h>

#if DUTY_CYCLE
#include <esp_timer.h>
// logDutyCycle is defined in main.cpp - declare as extern
extern void logDutyCycle(char event_type, uint64_t duration_us, bool success);
#endif

bool connectToMQTT(bool enableTLS) {
  #if MQTT_DEBUG
  // Serial.println("Attempting MQTT connection...");
  // print debug info
  Serial.print("WiFi status: ");
  Serial.println(isWifiConnected() ? "Connected" : "Disconnected");
  #endif 
  
  #if DUTY_CYCLE
  uint64_t mqtt_start_us = esp_timer_get_time();
  #endif
  
  bool success = false;

  if (enableTLS) {
    success = mqttClient.connect("ESP32Client", MQTT_USER, MQTT_PSWD);
  }
  else{
    success = mqttClient.connect("ESP32Client");
  }
  
  #if DUTY_CYCLE
  uint64_t mqtt_duration_us = esp_timer_get_time() - mqtt_start_us;
  logDutyCycle('M', mqtt_duration_us, success);
  #endif
  
  #if MQTT_DEBUG
  if (success) {
    Serial.println("MQTT connected successfully on first attempt");
  } 
  else {
    Serial.print("MQTT connection failed on first attempt, state=");
    Serial.println(mqttClient.state());
    switch(mqttClient.state()) {
        case -4: Serial.println("MQTT_CONNECTION_TIMEOUT"); break;
        case -3: Serial.println("MQTT_CONNECTION_LOST"); break;
        case -2: Serial.println("MQTT_CONNECT_FAILED"); break;
        case -1: Serial.println("MQTT_DISCONNECTED"); break;
        case 1: Serial.println("MQTT_CONNECT_BAD_PROTOCOL"); break;
        case 2: Serial.println("MQTT_CONNECT_BAD_CLIENT_ID"); break;
        case 3: Serial.println("MQTT_CONNECT_UNAVAILABLE"); break;
        case 4: Serial.println("MQTT_CONNECT_BAD_CREDENTIALS"); break;
        case 5: Serial.println("MQTT_CONNECT_UNAUTHORIZED"); break;
      }
  }
  #endif 
  return success;
}