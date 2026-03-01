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
#define IMU_READ_INTERVAL_MS 10 // 10 ms = 100 Hz 


enum class SafetyEvent { 
  NONE,
  FREEFALL,
  MEDIUM_IMPACT,
  HEAVY_IMPACT,
  HIGH_ROTATION_AND_ACC // high rotation and high acceleration is more indicative of a safety event
};

// keep out of namespace b/c hardware related
bool setupIMU();


namespace IMUTaskManager { 
  // state container : 
  // - keep sending alerts after detected once and it is not CLEARED 
  struct State { 
    bool fallActive; 
    AlertPayload originalFallAlert; // keep track of last alert payload to resend if still active

    // IMU analysis state
    imuData latestIMU;
    float accelWindow[WINDOW_SIZE]; // circular buffer to store recent acceleration magnitudes
    float gyroWindow[WINDOW_SIZE]; // circular buffer to store recent gyro magnitudes
    int windowIndex; // index for circular buffer

    // dependencies - pointers to avoid copies 
    QueueHandle_t imuQueue; // already a pointer
    QueueHandle_t gpsQueue; // already a pointer
    QueueHandle_t alertPublishQueue; // already a pointer
    EventGroupHandle_t mqttPublishEventGroup;
    EventGroupHandle_t gpsEventGroup;
  };


  // public 
  void init(State& s, 
    QueueHandle_t imuQueue, 
    QueueHandle_t gpsQueue, 
    QueueHandle_t alertPublishQueue, 
    EventGroupHandle_t mqttPublishEventGroup, 
    EventGroupHandle_t gpsEventGroup);

  void run(State& s); // main loop for IMU task, never returns

  // access functions for state
  bool isFallActive(State& s);
  void clearFallActive(State& s); // to be called when cancel button is pressed to clear fall state and stop sending alerts
  imuData getLatestIMU(State& s);

  // IMU operations
  void readIMU(State& s, imuData &data);
  SafetyEvent analyzeIMUData(State& s, const imuData &data); 
  // use const for read only, reference (no copy)

// IMU task manager namespace

}

#endif 
