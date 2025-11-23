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


#endif