#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>

struct CoreData { 
    // IMU data
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;

    // GPS data
    double latitude;
    double longitude;
    double altitude; // in meters
    float hdop;      // horizontal dilution of precision
    int satellites;  // number of satellites in view
    String dateTime; // date and time as string
    String eventType;
};

struct AirQualityData {
    // PM 
    float pm10_env;
    float pm25_env;
    float pm100_env;
    float aqi_pm25_us;
    float aqi_pm100_us;
    // ENS
    float temp;
    float humidity;
    float eco2;
    float tvoc;
    float aqi;
    String eventType;
    String dateTime;
     
};

struct NoiseData {
    float decibelLevel; // in dB 
    String eventType;   
    String dateTime;
};

struct TimeSync {
    char lastDateTime[20]; // last date/time string from GPS in format "YYYY/MM/DD,HH:MM:SS"
    TickType_t lastSyncTicks;
    bool hasValidSync; // whether we have a valid time sync from GPS
}; // adding this for graceful degradation of GPS time sync in case of signal loss, so we can still have somewhat accurate timestamps for data even without GPS fix


const CoreData EMPTY_COREDATA = {
    0.0, 0.0, 0.0,  // accelX, accelY, accelZ
    0.0, 0.0, 0.0,  // gyroX, gyroY, gyroZ
    0.0, 0.0, 0.0,  // latitude, longitude, altitude
    0.0,            // hdop
    0,              // satellites
    "",             // dateTime
    ""              // eventType
};

#endif