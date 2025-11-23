#ifndef MPU_SETUP_H
#define MPU_SETUP_H

#include <Adafruit_MPU6050.h> //include so compiler knows this library
#include "data_structs.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

//function declarations
void setupMPU(Adafruit_MPU6050 &mpu);
void readIMUData(Adafruit_MPU6050 &mpu, CoreData &data);

#endif 