#include "test.h"

bool serializeIMULog(const IMULogPayload& log, char* buffer, size_t bufferSize) {
  int len = snprintf(buffer, bufferSize,
    "{\"ts\":%lu,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
    "\"r_acc\":%.3f,\"r_gyro\":%.1f,\"event\":%u,\"state\":%u,\"jerk\":%.1f,"
    "\"freefall\":%d,\"horizontal\":%d,\"motionless\":%d}",
    log.timestamp_ms,
    log.accel_x, log.accel_y, log.accel_z,
    log.gyro_x, log.gyro_y, log.gyro_z,
    log.resultant_acc, log.resultant_gyro,
    log.safetyState, log.detectedEvent, log.jerk,
    log.is_freefall ? 1 : 0,
    log.is_horizontal ? 1 : 0,
    log.is_motionless ? 1 : 0);
  return (len < bufferSize);
}