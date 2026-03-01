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
    EventGroupHandle_t mqttPublishEventGroup) {
      s.fallActive = false; // init fall state to false
      memset(&s.originalFallAlert, 0, sizeof(AlertPayload));
      memset(&s.latestIMU, 0, sizeof(imuData));

      s.gpsQueue = gpsQueue;
      s.imuQueue = imuQueue;
      s.alertPublishQueue = alertPublishQueue;
      s.gpsEventGroup = gpsEventGroup;
      s.mqttPublishEventGroup = mqttPublishEventGroup;

      // Initialize windows (cannot init accel as 0 b/c that would be free fall case)
      for (int i = 0; i < WINDOW_SIZE; i++) {
        s.accelWindow[i] = 1.0;
        s.gyroWindow[i] = 0.0;
      }
      s.windowIndex = 0;
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
    #if IMU_DEBUG == 2
    Serial.println("Analyzing IMU data...");
    #endif
    // add to circular buffer
    s.accelWindow[s.windowIndex] = data.resultant_acc;
    s.gyroWindow[s.windowIndex] = data.resultant_gyro;
    s.windowIndex = (s.windowIndex + 1) % WINDOW_SIZE; // find next index in circular buffer

    // count samples exceeding thresholds
    int heavyImpactCount = 0;
    // int highRotationCount = 0;
    int highAccRotationCount = 0; // combined condition for high rotation and high acceleration
    int mediumImpactCount = 0;
    int freefallCount = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
      
      if (s.accelWindow[i] >= HEAVY_IMPACT_THRESHOLD_G) {
        heavyImpactCount++;
      }
      else if (s.accelWindow[i] >= MEDIUM_IMPACT_THRESHOLD_G) {
        mediumImpactCount++;
      }

      // count samples with BOTH impact and rotation
      if (s.accelWindow[i] >= MEDIUM_IMPACT_THRESHOLD_G && s.gyroWindow[i] >= ROTATION_THRESHOLD_DEG_S) {
        highAccRotationCount++;
      }
      
      if (s.accelWindow[i] <= FREEFALL_THRESHOLD_G) {
        freefallCount++;
      }
    }

    // check that counts exceed required number of samples in the buffer to be considered
    if (heavyImpactCount >= SUSTAINED_THRESHOLD) {
      #if IMU_DEBUG == 1 
      Serial.println("HEAVY IMPACT.");
      #endif
      return SafetyEvent::HEAVY_IMPACT;
    }
    else if (mediumImpactCount >= SUSTAINED_THRESHOLD) {
      #if IMU_DEBUG == 1 
      Serial.println("MEDIUM IMPACT.");
      #endif
      return SafetyEvent::MEDIUM_IMPACT;
    }

    else if (highAccRotationCount >= SUSTAINED_THRESHOLD) { 
      return SafetyEvent::HIGH_ROTATION_AND_ACC;
    }

    else if (freefallCount >= SUSTAINED_THRESHOLD) {
      #if IMU_DEBUG == 1 
      Serial.println("FREEFALL.");
      #endif
      return SafetyEvent::FREEFALL;
    }
    else {
      #if IMU_DEBUG == 1 
      Serial.println("NO IMU SAFETY EVENT.");
      #endif
      return SafetyEvent::NONE;
    }
  }

  imuData getLatestIMUData(State& s) {
    return s.latestIMU;
  }

  void run(State& s){ // main loop for IMU task, never returns
    imuData data; 

    while (true) {
      readIMU(s, data); // read and store latest IMU data in state
      if (s.imuQueue != NULL) { 
        // push into queue for heartbeat to use
        xQueueOverwrite(s.imuQueue, &data);
      }

      SafetyEvent event = analyzeIMUData(s, data);

      // check for a new fall 
      if (event != SafetyEvent::NONE && !s.fallActive && !isCancelActive()) {
        AlertPayload alertToSend;
        alertToSend.event = FALL_IMPACT;
        attachGPSToAlert(alertToSend, s.gpsEventGroup, s.gpsQueue);
        alertToSend.fall_detection = 1;
        alertToSend.noise_db = 0.0;
        
        setFallDetected(s, alertToSend);
        sendAlert(s, alertToSend);
      }

      // keep sending if fall active 
      else if (s.fallActive && !isCancelActive()) { 
        AlertPayload alertToResend;
        alertToResend.event = FALL_IMPACT;
        getOriginalFallGPSData(s, alertToResend); // get original GPS data from when fall was first detected
        alertToResend.fall_detection = 1;
        alertToResend.noise_db = 0.0;

        sendAlert(s, alertToResend);

      }

      vTaskDelay(pdMS_TO_TICKS(IMU_READ_INTERVAL_MS)); // delay before next read      
    }
  }

  // access functions for state
  bool isFallActive(State& s) {
    return s.fallActive;
  }

  void clearFallActive(State& s){
    s.fallActive = false;
    memset(&s.originalFallAlert, 0, sizeof(AlertPayload)); // clear original fall alert data
  } // to be called when cancel button is pressed to clear fall state and stop sending alerts

}