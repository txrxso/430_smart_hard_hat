#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

// FILE DEFINES MACROS TO ENABLE TESTING AND LOGGING FOR VERIFICATION
#define MQTT_TOPIC_IMU_DAQ "igen430/shh/imu_test"
#define ENABLE_IMU_LOGGING 0
#define DUTY_CYCLE 0 // set to 1 to enable tx_duration measurements for duty cycle calculation

#if ENABLE_IMU_LOGGING
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

#if DUTY_CYCLE
#include <esp_timer.h>

struct DutyCycleStats {
  char event_type;            // 'H'=Heartbeat, 'A'=Alert, 'W'=WiFi, 'M'=MQTT
  uint32_t esp32_timestamp_ms; // millis() when event occurred
  uint32_t duration_us;        // Duration of transmission in microseconds
  bool success;                // Was transmission successful
};

extern QueueHandle_t dutyCycleQueue; // queue for sending duty cycle stats from tasks to logger task
#define DC_QUEUE_SIZE 50 

// helper to log without blocking main task execution 
inline void logDutyCycle(char event_type, uint64_t duration_us, bool success) {
  DutyCycleStats stats = {event_type, (uint32_t)millis(), (uint32_t)duration_us, success};
  if (dutyCycleQueue != NULL) {
    xQueueSend(dutyCycleQueue, &stats, 0); // non-blocking send
  }
}

// task to drain queue and print stats to serial
inline void dutyCycleLogTask(void *parameter) { 
  DutyCycleStats stats;
  
  // Startup indicator
  Serial.println("DUTY_CYCLE_TASK_STARTED");
  
  while (true) {
    if (xQueueReceive(dutyCycleQueue, &stats, pdMS_TO_TICKS(100)) == pdTRUE) {
      // CSV format: event_type,esp32_timestamp_ms,duration_us,success
      Serial.printf("%c,%lu,%lu,%d\n", stats.event_type, stats.esp32_timestamp_ms, 
                    stats.duration_us, stats.success ? 1 : 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // fast drain
  }
}

inline void startDutyCycleTask() { 
    dutyCycleQueue = xQueueCreate(DC_QUEUE_SIZE, sizeof(DutyCycleStats));
    xTaskCreatePinnedToCore(
        dutyCycleLogTask,
        "DutyCycleLogTask",
        2048,
        NULL,
        1,  // increased from 0 - needs to run to drain queue
        NULL,
        1   // pin to core 1
    );
    Serial.println("DUTY_CYCLE_START"); // marker for Python script
}

#endif


#endif // for header