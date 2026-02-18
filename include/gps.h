#ifndef GPS_H
#define GPS_H

// for GPS
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Preferences.h> 

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> 

#include "data.h"
#include "debug.h"
#include "gps_events.h"

// UART pins
#define RXD2 16 
#define TXD2 17

// baud rate
#define GPS_BAUD 9600

// configuration
#define NVS_SAVE_PERIOD 5 // in minutes, how often to save GPS coordinates to NVS


struct TimeSync {
    char lastDateTime[20]; // last date/time string from GPS in format "YYYY/MM/DD,HH:MM:SS"
    TickType_t lastSyncTicks;
    bool hasValidSync; // whether we have a valid time sync from GPS
}; // adding this for graceful degradation of GPS time sync in case of signal loss, so we can still have somewhat accurate timestamps for data even without GPS fix


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