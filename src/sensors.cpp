#include "sensors.h"  // include its own header

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
Adafruit_MPU6050 mpu;

// internal state 
static imuData latestIMU;
static unsigned long freefallStart = 0;
static unsigned long eventStart = 0;
static float rAcc = 0; // resultant acceleration
static float rGyro = 0; // resultant gyro

static float accelWindow[WINDOW_SIZE] = {0};
static float gyroWindow[WINDOW_SIZE] = {0};
static int windowIndex = 0;

static float getResultant(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

void setupGPS() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

bool gpsHasFix() {
  return gps.location.isValid() && gps.satellites.value() >= 0;
}


bool gpsRead(gpsData &data) { 
  // update gps data structure with latest values from gps and return true if location updated
  // timeout after 1 second if no new data 

  unsigned long start = millis();

  while (true) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // check if location was updated 
    if (gps.location.isUpdated()) {
      // update values
      data.latitude = gps.location.lat();
      data.longitude = gps.location.lng();
      data.altitude = gps.altitude.meters();
      data.hdop = gps.hdop.value() / 100.0;
      data.satellites = gps.satellites.value();

      if (gps.date.isValid() && gps.time.isValid()) {
        snprintf(data.dateTime, sizeof(data.dateTime),
                 "%04d/%02d/%02d,%02d:%02d:%02d",
                 gps.date.year(),
                 gps.date.month(),
                 gps.date.day(),
                 gps.time.hour(),
                 gps.time.minute(),
                 gps.time.second());
      } 

      else {
        snprintf(data.dateTime, sizeof(data.dateTime), "Invalid");
      }

      return true; // exit early if updated

    }

    if (millis() - start > 1000) {
      // timeout after 1 second
      return false;
    }

  } // end of while loop

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
  // add to circular buffer
  accelWindow[windowIndex] = data.resultant_acc;
  gyroWindow[windowIndex] = data.resultant_gyro;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE; // find next index in circular buffer

  // count samples exceeding thresholds
  int heavyImpactCount = 0;
  int highRotationCount = 0;
  int mediumImpactCount = 0;
  int freefallCount = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (accelWindow[i] >= HEAVY_IMPACT_THRESHOLD_G) {
      heavyImpactCount++;
    }
    else if (accelWindow[i] >= MEDIUM_IMPACT_THRESHOLD_G) {
      mediumImpactCount++;
    }
    if (gyroWindow[i] >= ROTATION_THRESHOLD_DEG_S) {
      highRotationCount++;
    }
    if (accelWindow[i] <= FREEFALL_THRESHOLD_G) {
      freefallCount++;
    }
  }

  // check that counts exceed required number of samples in the buffer to cbe considered
  if (heavyImpactCount >= SUSTAINED_THRESHOLD) {
    return SafetyEvent::HEAVY_IMPACT;
  }
  else if (mediumImpactCount >= SUSTAINED_THRESHOLD) {
    return SafetyEvent::MEDIUM_IMPACT;
  }
  else if (highRotationCount >= SUSTAINED_THRESHOLD) {
    return SafetyEvent::HIGH_ROTATION;
  }
  else if (freefallCount >= SUSTAINED_THRESHOLD) {
    return SafetyEvent::FREEFALL;
  }
  else {
    return SafetyEvent::NONE;
  }
}