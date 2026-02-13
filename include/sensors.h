#ifndef SENSORS_H
#define SENSORS_H

// for GPS
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
// for IMU
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
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

/* 
1. Freefall detection (<0.3G for 100 ms)
2. Impact detection (> X G) 
3. Direct impact (> X G or rotation > 250 deg/s)
*/

#define FREEFALL_THRESHOLD_G 0.3
#define FREEFALL_DURATION_MS 100 // ms
#define MEDIUM_IMPACT_THRESHOLD_G 7.0
#define HEAVY_IMPACT_THRESHOLD_G 10.0
#define ROTATION_THRESHOLD_DEG_S 250.0
#define WINDOW_SIZE 3 // number of samples to consider for moving average; keep small
#define SUSTAINED_THRESHOLD 2 // number of consecutive samples exceeding threshold to consider valid event

enum class SafetyEvent { 
  NONE,
  FREEFALL,
  MEDIUM_IMPACT,
  HEAVY_IMPACT,
  HIGH_ROTATION
};

void setupGPS();
bool gpsHasFix(); 
bool gpsRead(gpsData &data);
void computeDateTime(char* buffer, size_t bufferSize, const char* baseDateTime, TickType_t baseSyncTicks); // helper to compute date/time string from GPS data for timestamping
bool setupIMU();
void readIMU(imuData &data);
SafetyEvent analyzeIMUData(const imuData &data); // use const for read only, reference (no copy)
imuData getLatestIMUData();

#endif 
