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

// ============================================
// THRESHOLDS AND PARAMETERS FOR IMU ANALYSIS
// ============================================
// severe impact (immediate alert and threshold based)
#define SEVERE_IMPACT_THRESHOLD_G 8.0

// state machine thresholds 
#define MEDIUM_IMPACT_THRESHOLD_G 4.0
#define FREEFALL_THRESHOLD_G 0.3
#define JERK_THRESHOLD_G_PER_S 65.0 // TODO: REPLACE WITH ACTUAL VALUE FROM TEST DATA
#define MOTIONLESS_ACC_THRESHOLD 0.3 

// timing constraints 
#define MIN_FREEFALL_DURATION_MS 100 // minimum freefall duration to be valid
#define MAX_FREEFALL_TO_IMPACT_MS 500 // max time from freefall to impact
#define IMU_READ_INTERVAL_MS 10 // 10 ms = 100 Hz 
#define MOTIONLESS_THRESHOLD_MS 2000 // time after impact to consider motionless for fall state
#define POST_IMPACT_OBS_DURATION_MS 1000 // time after impact to observe for freefall or motionless state
#define ROTATION_THRESHOLD_DEG_S 500.0 // high rotation threshold
#define MOTIONLESS_GYRO_THRESHOLD_DEG_S 50.0 // low gyro for motionless detection

// for immediate alerting considerations
#define WINDOW_SIZE 3 // number of samples to consider for moving average; keep small
#define SUSTAINED_THRESHOLD 2 // number of consecutive samples exceeding threshold to consider valid event

// ============================================
// OUTPUT OF IMU ANALYSIS (PUBLIC)
// ============================================
enum class SafetyEvent {  // what is returned to send alert
  NONE,
  FALL,
  DIRECT_IMPACT, // sudden impact without freefall
  HEAVY_IMPACT, // very high acceleration, immediate alert 
};

// ============================================
// STATE MACHINE STATES (INTERNAL)
// ============================================
enum class InternalSafetyState { 
  NORMAL,
  FREEFALL,
  IMPACT,
  POST_IMPACT,
  INJURY_LIKELY,
  RECOVERED
};


// ============================================
// HARDWARE SETUP
// ============================================
// keep out of namespace b/c hardware related
bool setupIMU();

// ============================================
// IMU Task Manager - All state and operations
// ============================================
namespace IMUTaskManager { 
  // INTERNAL DETECTION STATE - not exposed outside task
  struct InternalDetectionState { 
    InternalSafetyState currentState;
    SafetyEvent detectedEvent;

    // timing 
    uint32_t freefallStartTime;
    uint32_t impactStartTime;
    uint32_t motionlessStartTime;
    uint32_t lastReadTime;

    // sensor history for jerk calculation: 
    // - use 3 samples to calculate (for a bit of smoothing)
    float prevResultantAcc;
    uint32_t prevAccTime; // timestamp of previous acceleration reading for jerk calculation
    float prev2ResultantAcc; // 2 samples back
    uint32_t prev2AccTime;
    float prev3ResultantAcc; // 3 samples back
    uint32_t prev3AccTime;

    // orientation tracking 
    float preEventOrientation[3]; // x, y, z orientation before event for comparison to post event orientation to detect high rotation
  };

  // STATE CONTAINER 
  // - keep sending alerts after detected once and it is not CLEARED 
  struct State { 
    bool fallActive; 
    AlertPayload originalFallAlert; // keep track of last alert payload to resend if still active

    // detection state 
    InternalDetectionState detectionState;

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

    #if ENABLE_IMU_LOGGING
    QueueHandle_t imuLoggingQueue; // for logging raw IMU data and events for offline analysis
    #endif
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
