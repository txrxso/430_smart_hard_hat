#include "wifi_config.h"

void connectToWifi() {
    
    WiFi.begin(HOTSPOT_SSID, HOTSPOT_PSWD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to Wifi...");
        Serial.println(WiFi.status());
    }
}


void connectToWifiEnterprise() {

    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WPA2_AUTH_PEAP, WIFI_EAP_ID, WIFI_USER, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(3000);
        Serial.println("Connecting to Enterprise Wifi...");
        Serial.println(WiFi.status());
    }

}

bool isWifiConnected() { 
    return WiFi.status() == WL_CONNECTED;
}