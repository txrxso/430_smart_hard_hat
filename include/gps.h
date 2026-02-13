#ifndef GPS_H
#define GPS_H

// for GPS
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Preferences.h> 

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h> 

#include "data.h"
#include "debug.h"

// UART pins
#define RXD2 16 
#define TXD2 17

// baud rate
#define GPS_BAUD 9600

// event group for fresh GPS read requests
extern EventGroupHandle_t gpsEventGroup;
#define GPS_READ_REQUEST_BIT (1 << 0) 
#define GPS_READ_SUCCESS_BIT (1 << 1)

// configuration
#define NVS_SAVE_PERIOD 5 // in minutes, how often to save GPS coordinates to NVS

// low-level helpers
void setupGPS();
bool gpsHasFix(); 
bool gpsRead(gpsData &data);
// helper to compute date/time string from GPS data for timestamping
void computeDateTime(char* buffer, size_t bufferSize, 
    const char* baseDateTime, TickType_t baseSyncTicks); 

// gps task manager (namespace)
namespace GPSTaskManager { 

    // state container : GPSTaskManager::State
    struct State {
        gpsData lastValidGPS; // store last valid GPS data for timestamping and fallback
        TimeSync timesync; // for keeping track of time since last GPS sync
        TickType_t lastNVSSave; 
        TickType_t lastPeriodicRead;

        // dependencies - pointers to avoid copies
        Preferences* gpsPreferences;
        QueueHandle_t gpsQueue; // already a pointer
        EventGroupHandle_t gpsEventGroup;
    };

    // public API
    void init(State& s,  
        QueueHandle_t gpsQueue, 
        EventGroupHandle_t gpsEventGroup);
    void run(State& s); // main loop (never returns)
}


#endif