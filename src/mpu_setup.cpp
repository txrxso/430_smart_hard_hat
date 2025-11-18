#include "mpu_setup.h"

#include "mpu_setup.h"  // include its own header
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


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


String createPayload(Adafruit_MPU6050 &mpu) {
  // get new sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // create payload
  String payload = "{";
  payload += "\"accel_x\":" + String(a.acceleration.x, 2) + ",";
  payload += "\"accel_y\":" + String(a.acceleration.y, 2) + ",";
  payload += "\"accel_z\":" + String(a.acceleration.z, 2) + ",";
  payload += "\"gyro_x\":" + String(g.gyro.x, 2) + ",";
  payload += "\"gyro_y\":" + String(g.gyro.y, 2) + ",";
  payload += "\"gyro_z\":" + String(g.gyro.z, 2);
  payload += "}";

  return payload;
}


