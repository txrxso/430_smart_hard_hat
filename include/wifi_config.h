#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include "secrets.h"  // This file should define the credentials required to connect to eduroam or the desired wifi

// function declarations
void connectToWifi();
void connectToWifiEnterprise();
bool isWifiConnected();

#endif 
