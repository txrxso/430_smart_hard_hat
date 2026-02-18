#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// for IMU
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "data.h"
#include "debug.h"

/* 
1. Freefall detection (<0.3G for 100 ms)
2. Impact detection (> X G) 
3. Direct impact (> X G or rotation > 250 deg/s)
*/

#define FREEFALL_THRESHOLD_G 0.3
#define FREEFALL_DURATION_MS 100 // ms
#define MEDIUM_IMPACT_THRESHOLD_G 4.0
#define HEAVY_IMPACT_THRESHOLD_G 10.0
#define ROTATION_THRESHOLD_DEG_S 500
#define WINDOW_SIZE 3 // number of samples to consider for moving average; keep small
#define SUSTAINED_THRESHOLD 2 // number of consecutive samples exceeding threshold to consider valid event

enum class SafetyEvent { 
  NONE,
  FREEFALL,
  MEDIUM_IMPACT,
  HEAVY_IMPACT,
  HIGH_ROTATION_AND_ACC // high rotation and high acceleration is more indicative of a safety event
};

bool setupIMU();
void readIMU(imuData &data);
SafetyEvent analyzeIMUData(const imuData &data); // use const for read only, reference (no copy)
imuData getLatestIMUData();

#endif 
