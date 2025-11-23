#include "mpu_setup.h"

void setupMPU(Adafruit_MPU6050 &mpu) {
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    //Serial.println("MPU6050 Found!");
  
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);

    // keep for later; TO DO: trigger events only when MPU detects motion
    // to avoid constant polling
    mpu.setMotionDetectionThreshold(1); // adjust later as needed
    mpu.setMotionDetectionDuration(20); // adjust later as needed
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true); 
    // TO DO: add interrupt handling for main.cpp later

    delay(100);
  }


void readIMUData(Adafruit_MPU6050 &mpu, CoreData &data) {
    // 1. Read MPU data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 2. Modify the referenced 'data' object directly
    data.accelX = a.acceleration.x;
    data.accelY = a.acceleration.y;
    data.accelZ = a.acceleration.z;

    data.gyroX = g.gyro.x;
    data.gyroY = g.gyro.y;
    data.gyroZ = g.gyro.z;
    
}


