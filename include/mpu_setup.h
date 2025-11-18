#ifndef MPU_SETUP_H
#define MPU_SETUP_H

#include <Adafruit_MPU6050.h> //include so compiler knows this library

//function declarations
void setupMPU(Adafruit_MPU6050 &mpu);
String createPayload(Adafruit_MPU6050 &mpu);

#endif 
