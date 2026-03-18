#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

// FILE DEFINES MACROS TO ENABLE TESTING AND LOGGING FOR VERIFICATION
#define MQTT_TOPIC_IMU_DAQ "igen430/shh/imu_test"
#define ENABLE_IMU_LOGGING 1

struct IMULogPayload { 
    uint32_t timestamp_ms; 

    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float resultant_acc;
    float resultant_gyro;

    // detection state values
    uint8_t safetyState; // value of InternalSafetyState enum
    uint8_t detectedEvent; // value of SafetyEvent enum
    float jerk; // calculated resultant jerk 

    // state machine 
    bool is_freefall;
    bool is_horizontal;
    bool is_motionless;
};

bool serializeIMULog(const IMULogPayload& log, char* buffer, size_t bufferSize);

#endif