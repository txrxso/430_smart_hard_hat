#include "imu.h"  // include its own header

Adafruit_MPU6050 mpu;

// internal state 
static imuData latestIMU;
static unsigned long freefallStart = 0;
static unsigned long eventStart = 0;
static float rAcc = 0; // resultant acceleration
static float rGyro = 0; // resultant gyro

static float accelWindow[WINDOW_SIZE] = {1.0,1.0,1.0}; // cannot init as 0 b/c that would be free fall case
static float gyroWindow[WINDOW_SIZE] = {0.0,0.0,0.0};
static int windowIndex = 0;

static float getResultant(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}


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

imuData getLatestIMUData() {
  return latestIMU;
}


void readIMU(imuData &data) {
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
  latestIMU = data;
}

SafetyEvent analyzeIMUData(const imuData &data) {
  #if IMU_DEBUG == 2
  Serial.println("Analyzing IMU data...");
  #endif
  // add to circular buffer
  accelWindow[windowIndex] = data.resultant_acc;
  gyroWindow[windowIndex] = data.resultant_gyro;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE; // find next index in circular buffer

  // count samples exceeding thresholds
  int heavyImpactCount = 0;
  // int highRotationCount = 0;
  int highAccRotationCount = 0; // combined condition for high rotation and high acceleration
  int mediumImpactCount = 0;
  int freefallCount = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    
    if (accelWindow[i] >= HEAVY_IMPACT_THRESHOLD_G) {
      heavyImpactCount++;
    }
    else if (accelWindow[i] >= MEDIUM_IMPACT_THRESHOLD_G) {
      mediumImpactCount++;
    }

    // count samples with BOTH impact and rotation
    if (accelWindow[i] >= MEDIUM_IMPACT_THRESHOLD_G && gyroWindow[i] >= ROTATION_THRESHOLD_DEG_S) {
      highAccRotationCount++;
    }
    
    if (accelWindow[i] <= FREEFALL_THRESHOLD_G) {
      freefallCount++;
    }
  }

  // check that counts exceed required number of samples in the buffer to cbe considered
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