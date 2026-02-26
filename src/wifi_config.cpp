#include "wifi_config.h"

void connectToWifi() {
    int attempts = 0;
    WiFi.begin(HOTSPOT_SSID, HOTSPOT_PSWD);
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, IPAddress(8,8,8,8));
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
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