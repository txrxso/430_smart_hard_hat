#include "imu.h"  // include its own header
#include "can.h"
#include "button.h"

// Hardware object (global)
Adafruit_MPU6050 mpu;

bool setupIMU(){ 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }

  // configure MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);

  return true;
}


static float rAcc = 0; // resultant acceleration
static float rGyro = 0; // resultant gyro

// =======================================
// IMU Task Manager - All state and operations
// =======================================
namespace IMUTaskManager { 
  // public 
  void init(State& s, 
    QueueHandle_t imuQueue, 
    QueueHandle_t gpsQueue, 
    QueueHandle_t alertPublishQueue, 
    EventGroupHandle_t gpsEventGroup,
    EventGroupHandle_t mqttPublishEventGroup,
    VibeManager::State* vibeState) {
      s.fallActive = false; // init fall state to false
      s.lastAlertSendTime = 0; // initialize alert send timestamp
      memset(&s.originalFallAlert, 0, sizeof(AlertPayload));
      memset(&s.latestIMU, 0, sizeof(imuData));

      s.gpsQueue = gpsQueue;
      s.imuQueue = imuQueue;
      s.alertPublishQueue = alertPublishQueue;
      s.gpsEventGroup = gpsEventGroup;
      s.mqttPublishEventGroup = mqttPublishEventGroup;
      s.vibeState = vibeState;

      #if ENABLE_IMU_LOGGING
      s.imuLoggingQueue = NULL; // will be set from main.cpp
      #endif

      // Initialize windows (cannot init accel as 0 b/c that would be free fall case)
      for (int i = 0; i < WINDOW_SIZE; i++) {
        s.accelWindow[i] = 1.0;
        s.gyroWindow[i] = 0.0;
      }
      s.windowIndex = 0;
      
      // Initialize detection state
      s.detectionState.currentState = InternalSafetyState::NORMAL;
      s.detectionState.detectedEvent = SafetyEvent::NONE;
      s.detectionState.freefallStartTime = 0;
      s.detectionState.impactStartTime = 0;
      s.detectionState.motionlessStartTime = 0;
      s.detectionState.lastReadTime = millis();
      s.detectionState.prevResultantAcc = 1.0;
      s.detectionState.prevAccTime = millis();
      s.detectionState.prev2ResultantAcc = 1.0;
      s.detectionState.prev2AccTime = millis();
      s.detectionState.prev3ResultantAcc = 1.0;
      s.detectionState.prev3AccTime = millis();
      s.detectionState.preEventOrientation[0] = 0.0;
      s.detectionState.preEventOrientation[1] = 0.0;
      s.detectionState.preEventOrientation[2] = 1.0;
    }

  // helper to store fall data
  static void setFallDetected(State& s, const AlertPayload& firstAlert) { 
    s.fallActive = true;
    memcpy(&s.originalFallAlert, &firstAlert, sizeof(AlertPayload));
  }

  // helper to calculate resultant
  static float getResultant(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
  }

  // helper to get original fall GPS data for resending alerts if still active 
  // (lat, lon, datetime) from the original fall alert for resending if still in active fall state
  static void getOriginalFallGPSData(State& s, AlertPayload& alert) {
    alert.latitude = s.originalFallAlert.latitude;
    alert.longitude = s.originalFallAlert.longitude;
    alert.altitude = s.originalFallAlert.altitude;
    strncpy(alert.dateTime, s.originalFallAlert.dateTime, sizeof(alert.dateTime));
  }

  // helper to send alert 
  static void sendAlert(State& s, AlertPayload& alert) {
    if (s.alertPublishQueue != NULL) { 
      BaseType_t result = xQueueSend(s.alertPublishQueue, &alert, pdMS_TO_TICKS(5));
      xEventGroupSetBits(s.mqttPublishEventGroup, PUBLISH_IMU_THRESHOLD_BIT); // signal MQTT publish for new alert
      #if IMU_DEBUG == 1
      printAlertPayload(alert);
      #endif 
    } 
    
    else {
      Serial.println("Alert publish queue is NULL, cannot send alert.");

    }
  }

  // helper to update history after jerk calculation
  static void updateJerkHistory(InternalDetectionState &d, float currentAcc, uint32_t currentTime) {
    d.prev3ResultantAcc = d.prev2ResultantAcc;
    d.prev3AccTime = d.prev2AccTime;
    d.prev2ResultantAcc = d.prevResultantAcc;
    d.prev2AccTime = d.prevAccTime;
    d.prevResultantAcc = currentAcc;
    d.prevAccTime = currentTime;
  }

  static float getJerk(float sample1, float sample2, float timeDelta_sec) {
    // sample 2 is the one ahead in time, sample 1 is the one behind in time
    if (timeDelta_sec <= 0) {
      return 0; // avoid division by zero, treat as no jerk
    }
    return (sample2 - sample1) / timeDelta_sec;
  }

  // helper to calculate jerk with individual time deltas
  static float calcJerk(InternalDetectionState &d, float currentAcc, uint32_t currentTime) {
    // Calculate individual jerks with their actual time deltas
    float dt1 = (currentTime - d.prevAccTime) / 1000.0;
    float dt2 = (d.prevAccTime - d.prev2AccTime) / 1000.0;
    float dt3 = (d.prev2AccTime - d.prev3AccTime) / 1000.0;
    
    // Avoid division by zero
    if (dt1 <= 0) dt1 = 0.01;
    if (dt2 <= 0) dt2 = 0.01;
    if (dt3 <= 0) dt3 = 0.01;
    
    float jerk1 = getJerk(d.prevResultantAcc, currentAcc, dt1);
    float jerk2 = getJerk(d.prev2ResultantAcc, d.prevResultantAcc, dt2);
    float jerk3 = getJerk(d.prev3ResultantAcc, d.prev2ResultantAcc, dt3);

    float peakJerk = max(max(abs(jerk1), abs(jerk2)), abs(jerk3));
    return peakJerk;
  }
  
  // Check if person is lying horizontally (gravity in x-y plane, not z)
  static bool checkPostImpactOrientation(float ax, float ay, float az) {
    float horizontal_acc = sqrt(ax*ax + ay*ay);
    float vertical_acc = abs(az);
    
    // Horizontal if gravity is mostly in x-y plane
    // Standing: az ≈ 1g, horizontal ≈ 0
    // Lying down: az ≈ 0, horizontal ≈ 1g
    return (horizontal_acc > 0.7 && vertical_acc < 0.5);
  }

  void readIMU(State& s, imuData &data) {
    #if IMU_DEBUG == 2 
    Serial.println("Reading IMU data...");
    #endif
    sensors_event_t a, g, temp; // read raw
    mpu.getEvent(&a, &g, &temp);

    data.accX = a.acceleration.x / 9.81;
    data.accY = a.acceleration.y / 9.81;
    data.accZ = a.acceleration.z / 9.81; // m/s^2 -> g's

    data.gyroX = g.gyro.x * 57.3;
    data.gyroY = g.gyro.y * 57.3;
    data.gyroZ = g.gyro.z * 57.3; // rad/s -> deg/s 

    data.resultant_acc = getResultant(data.accX, data.accY, data.accZ);
    data.resultant_gyro = getResultant(data.gyroX, data.gyroY, data.gyroZ);

    // store as latest
    s.latestIMU = data;
  }

  SafetyEvent analyzeIMUData(State& s, const imuData &data) {
    InternalDetectionState& d = s.detectionState;
    
    // Get current time
    uint32_t currentTime = millis();
    
    // Update window BEFORE state machine
    s.accelWindow[s.windowIndex] = data.resultant_acc;
    s.gyroWindow[s.windowIndex] = data.resultant_gyro;
    s.windowIndex = (s.windowIndex + 1) % WINDOW_SIZE;
    
    // Calculate jerk
    float jerk = calcJerk(d, data.resultant_acc, currentTime);
    
    #if IMU_DEBUG == 2
    Serial.printf("State: %d, Acc: %.2f, Jerk: %.1f, Gyro: %.1f\n", 
                  (int)d.currentState, data.resultant_acc, jerk, data.resultant_gyro);
    #endif
    
    // ========================================
    // TIER 1: SEVERE IMPACT - IMMEDIATE ALERT
    // ========================================
    if (data.resultant_acc > SEVERE_IMPACT_THRESHOLD_G) {
      #if IMU_DEBUG == 1
      Serial.printf("SEVERE IMPACT: %.2fg - IMMEDIATE ALERT\n", data.resultant_acc);
      #endif
      
      d.currentState = InternalSafetyState::INJURY_LIKELY;
      d.detectedEvent = SafetyEvent::HEAVY_IMPACT;
      updateJerkHistory(d, data.resultant_acc, currentTime);
      return SafetyEvent::HEAVY_IMPACT;
    }
    
    // ========================================
    // TIER 2: STATE MACHINE
    // ========================================
    
    switch (d.currentState) {
      
      case InternalSafetyState::NORMAL: {
        
        // Path A: Freefall detected (likely fall)
        if (data.resultant_acc < FREEFALL_THRESHOLD_G) {
          d.freefallStartTime = currentTime;
          d.detectedEvent = SafetyEvent::FALL;
          d.currentState = InternalSafetyState::FREEFALL;
          
          // Store pre-fall orientation
          d.preEventOrientation[0] = data.accX;
          d.preEventOrientation[1] = data.accY;
          d.preEventOrientation[2] = data.accZ;
          
          #if IMU_DEBUG == 1
          Serial.println("Freefall detected - monitoring for FALL");
          #endif
        }
        
        // Path B: Moderate impact with high jerk (direct impact)
        else if (data.resultant_acc > MEDIUM_IMPACT_THRESHOLD_G && 
                 jerk > JERK_THRESHOLD_G_PER_S) {
          
          // Window validation: check if sustained
          int impactCount = 0;
          for (int i = 0; i < WINDOW_SIZE; i++) {
            if (s.accelWindow[i] > MEDIUM_IMPACT_THRESHOLD_G) {
              impactCount++;
            }
          }
          
          // If sustained (2/3 samples high), it's real
          if (impactCount >= SUSTAINED_THRESHOLD) {
            d.impactStartTime = currentTime;
            d.detectedEvent = SafetyEvent::DIRECT_IMPACT;
            d.currentState = InternalSafetyState::IMPACT;
            
            #if IMU_DEBUG == 1
            Serial.printf("Direct impact: %.2fg, jerk: %.1f g/s (sustained: %d/%d)\n", 
                          data.resultant_acc, jerk, impactCount, WINDOW_SIZE);
            #endif
          }
        }
        
        // Path C: High rotation + moderate acceleration
        else if (data.resultant_gyro > ROTATION_THRESHOLD_DEG_S && 
                 data.resultant_acc > MEDIUM_IMPACT_THRESHOLD_G) {
          d.impactStartTime = currentTime;
          d.detectedEvent = SafetyEvent::DIRECT_IMPACT;
          d.currentState = InternalSafetyState::IMPACT;
          
          #if IMU_DEBUG == 1
          Serial.printf("Rotational impact: %.1f deg/s, %.2fg\n", 
                        data.resultant_gyro, data.resultant_acc);
          #endif
        }

        // Path D: REMOVED jerk-only detection (too prone to false positives from vibrations/shaking)
        // Jerk must now be combined with impact threshold (Path B) for validation
        
        break;
      }
      
      case InternalSafetyState::FREEFALL: {
        uint32_t freefallDuration = currentTime - d.freefallStartTime;
        
        // Freefall to Impact transition (WITH jerk validation)
        if (data.resultant_acc > MEDIUM_IMPACT_THRESHOLD_G && 
            jerk > JERK_THRESHOLD_G_PER_S) {
          
          // Validate timing
          if (freefallDuration >= MIN_FREEFALL_DURATION_MS && 
              freefallDuration <= MAX_FREEFALL_TO_IMPACT_MS) {
            d.impactStartTime = currentTime;
            d.currentState = InternalSafetyState::IMPACT;
            
            #if IMU_DEBUG == 1
            Serial.printf("FALL impact: freefall %lums → impact %.2fg\n", 
                          freefallDuration, data.resultant_acc);
            #endif
          } 
          else {
            // Invalid timing pattern (e.g., phone dropped)
            d.currentState = InternalSafetyState::RECOVERED;
            
            #if IMU_DEBUG == 1
            Serial.printf("Invalid freefall timing (%lums) - recovery\n", freefallDuration);
            #endif
          }
        }
        // Timeout - caught themselves, avoided fall
        else if (freefallDuration > MAX_FREEFALL_TO_IMPACT_MS) {
          d.currentState = InternalSafetyState::RECOVERED;
          
          #if IMU_DEBUG == 1
          Serial.println("Freefall timeout - caught self, recovery");
          #endif
        }
        
        break;
      }
      
      case InternalSafetyState::IMPACT: {
        uint32_t timeSinceImpact = currentTime - d.impactStartTime;
        
        // Wait brief period to observe post-impact behavior
        if (timeSinceImpact > POST_IMPACT_OBS_DURATION_MS) {
          d.currentState = InternalSafetyState::POST_IMPACT;
          d.motionlessStartTime = currentTime;
          
          #if IMU_DEBUG == 2
          Serial.println("Entering POST_IMPACT observation");
          #endif
        }
        
        break;
      }
      
      case InternalSafetyState::POST_IMPACT: {
        // Check orientation
        bool isHorizontal = checkPostImpactOrientation(data.accX, data.accY, data.accZ);
        
        // Check if motionless
        bool isMotionless = (abs(data.resultant_acc - 1.0) < MOTIONLESS_ACC_THRESHOLD && 
                             data.resultant_gyro < MOTIONLESS_GYRO_THRESHOLD_DEG_S);
        
        if (isHorizontal && isMotionless) {
          // Person is lying down and not moving - likely injured
          uint32_t stationaryDuration = currentTime - d.motionlessStartTime;
          
          if (stationaryDuration > MOTIONLESS_THRESHOLD_MS) {
            d.currentState = InternalSafetyState::INJURY_LIKELY;
            
            #if IMU_DEBUG == 1
            Serial.printf("INJURY CONFIRMED: %s - Horizontal & motionless for %lums\n",
                          (d.detectedEvent == SafetyEvent::FALL) ? "FALL" : "DIRECT_IMPACT",
                          stationaryDuration);
            #endif
            
            updateJerkHistory(d, data.resultant_acc, currentTime);
            return d.detectedEvent;
          }
        } 
        else if (!isMotionless) {
          // Person is moving - probably recovered
          d.currentState = InternalSafetyState::RECOVERED;
          
          #if IMU_DEBUG == 1
          Serial.println("Movement detected after impact - RECOVERED");
          #endif
        }
        else {
          // Not horizontal yet but motionless - reset timer
          d.motionlessStartTime = currentTime;
        }
        
        break;
      }
      
      case InternalSafetyState::INJURY_LIKELY: {
        // Stay in this state, continue returning detected event
        updateJerkHistory(d, data.resultant_acc, currentTime);
        return d.detectedEvent;
      }
      
      case InternalSafetyState::RECOVERED: {
        // Reset to normal
        d.currentState = InternalSafetyState::NORMAL;
        d.detectedEvent = SafetyEvent::NONE;
        
        #if IMU_DEBUG == 1
        Serial.println("State reset to NORMAL");
        #endif
        
        break;
      }
    }
    
    updateJerkHistory(d, data.resultant_acc, currentTime);
    return SafetyEvent::NONE;
  }

  imuData getLatestIMUData(State& s) {
    return s.latestIMU;
  }

  void run(State& s){ // main loop for IMU task, never returns
    imuData data; 

    #if ENABLE_IMU_LOGGING
    const uint32_t LOG_INTERVAL_MS = 10; // every 10 ms (100 Hz)
    const uint32_t LOG_INTERVAL_TICKS = pdMS_TO_TICKS(LOG_INTERVAL_MS);
    uint32_t lastLogTime = 0;
    bool firstLog = true;
    #endif

    while (true) {
      readIMU(s, data); // read and store latest IMU data in state
      if (s.imuQueue != NULL) { 
        // push into queue for heartbeat to use
        xQueueOverwrite(s.imuQueue, &data);
      }

      SafetyEvent event = analyzeIMUData(s, data);

      #if ENABLE_IMU_LOGGING  // do not alert, only send the raw payload
      // log values in payload format every interval 
      uint32_t currentTime = millis();
      if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {

        IMULogPayload logPayload;
        logPayload.accel_x = data.accX;
        logPayload.accel_y = data.accY;
        logPayload.accel_z = data.accZ;
        logPayload.gyro_x = data.gyroX;
        logPayload.gyro_y = data.gyroY;
        logPayload.gyro_z = data.gyroZ;
        logPayload.resultant_acc = data.resultant_acc;
        logPayload.resultant_gyro = data.resultant_gyro;
        logPayload.safetyState = static_cast<uint8_t>(s.detectionState.currentState);
        logPayload.detectedEvent = static_cast<uint8_t>(s.detectionState.detectedEvent);
        logPayload.jerk = calcJerk(s.detectionState, data.resultant_acc, currentTime);
        logPayload.is_freefall = (s.detectionState.currentState == InternalSafetyState::FREEFALL);
        logPayload.is_horizontal = checkPostImpactOrientation(data.accX, data.accY, data.accZ);
        logPayload.is_motionless = (abs(data.resultant_acc - 1.0) < MOTIONLESS_ACC_THRESHOLD && 
                                    data.resultant_gyro < MOTIONLESS_GYRO_THRESHOLD_DEG_S);
        logPayload.timestamp_ms = millis();
        
        // Only send if queue is initialized
        if (s.imuLoggingQueue != NULL) {
          BaseType_t result = xQueueSend(s.imuLoggingQueue, &logPayload, pdMS_TO_TICKS(5));
          if (result == pdTRUE) {
            if (firstLog) {
              Serial.printf("[IMU] First log sent to queue %p\n", s.imuLoggingQueue);
              Serial.printf("[IMU] Setting event bit 0x%02X on group %p\n", PUBLISH_IMU_LOG_BIT, s.mqttPublishEventGroup);
              firstLog = false;
            }
            xEventGroupSetBits(s.mqttPublishEventGroup, PUBLISH_IMU_LOG_BIT); // signal mqtt task to publish
          }
        } else {
          if (firstLog) {
            Serial.println("[IMU] ERROR: imuLoggingQueue is NULL!");
            firstLog = false;
          }
        }
        lastLogTime = currentTime;
      }

      #else // do normal alerting
      // check for a new fall 
      if (event != SafetyEvent::NONE && !s.fallActive && !isCancelActive()) {
        AlertPayload alertToSend;
        alertToSend.event = FALL_IMPACT;
        attachGPSToAlert(alertToSend, s.gpsEventGroup, s.gpsQueue);
        alertToSend.fall_detection = 1;
        alertToSend.noise_db = 0.0;
        
        setFallDetected(s, alertToSend);
        sendAlert(s, alertToSend);
        s.lastAlertSendTime = millis(); // initialize send timestamp
        
        // Trigger vibration pattern for fall/impact
        if (s.vibeState) {
          VibeManager::startPattern(*s.vibeState, VibePattern::FALL_IMPACT);
        }
      }

      // keep sending if fall active (every 1 second)
      else if (s.fallActive && !isCancelActive()) {
        unsigned long currentTime = millis();
        
        // Only resend every 1 second
        if (currentTime - s.lastAlertSendTime >= 1000) {
          AlertPayload alertToResend;
          alertToResend.event = FALL_IMPACT;
          getOriginalFallGPSData(s, alertToResend); // get original GPS data from when fall was first detected
          alertToResend.fall_detection = 1;
          alertToResend.noise_db = 0.0;

          sendAlert(s, alertToResend);
          s.lastAlertSendTime = currentTime; // update last send time
        }
      }
      #endif 

      vTaskDelay(pdMS_TO_TICKS(IMU_READ_INTERVAL_MS)); // delay before next read      
    }
  }

  // access functions for state
  bool isFallActive(State& s) {
    return s.fallActive;
  }

  void clearFallActive(State& s){
    s.fallActive = false;
    s.lastAlertSendTime = 0; // reset timer
    memset(&s.originalFallAlert, 0, sizeof(AlertPayload)); // clear original fall alert data
    s.detectionState.currentState = InternalSafetyState::NORMAL;
    s.detectionState.detectedEvent = SafetyEvent::NONE;
  } // to be called when cancel button is pressed to clear fall state and stop sending alerts

}