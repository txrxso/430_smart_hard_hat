#include "data.h"

const char* alertTypeToString(AlertType type) {
    switch (type) {
        case FALL_IMPACT:
            return "FALL_IMPACT";
        case NOISE_OVER_THRESHOLD:
            return "NOISE_OVER_THRESHOLD";
        case AIR_QUALITY_OVER_THRESHOLD:
            return "AIR_QUALITY_OVER_THRESHOLD";
        case MANUAL_ALERT:
            return "MANUAL_ALERT";
        case MANUAL_CLEAR:
            return "MANUAL_CLEAR";
        default:
            return "UNKNOWN_ALERT_TYPE";
    }
}

// helper to serialize AlertPayload to JSON 
bool serializeAP(const AlertPayload& alert, char* buffer, size_t bufferSize) { 

  int len = snprintf(buffer, bufferSize,
    "{\"event\":\"%s\",\"datetime\":\"%s\",\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,\"measurements\":[",
    alertTypeToString(alert.event),
    alert.dateTime,
    alert.latitude,
    alert.longitude,
    alert.altitude);

  if (len >= bufferSize) return false;

  // add measurements dynamically 
  bool first = true;
  for (int i = 0; i < 5; i++) {
    int m_len = snprintf(buffer + len, bufferSize - len,
      "%s{\"%s\":%.2f}",
      first ? "" : ",",
      alert.measurements[i].key,
      alert.measurements[i].value); 

      if (len + m_len >= bufferSize) return false;
      len += m_len; // append 
      first = false;
  }

  // close the measuremetns array and json 
  int endlen = snprintf(buffer + len, bufferSize - len, "]}");
  if (len + endlen >= bufferSize) return false;

  return true;
  
}

// helper to serialize HeartbeatPayload to string that can be parsed by backend as JSON
bool serializeHB(const hbPayload& heartbeat, char* buffer, size_t bufferSize) {

  int len = snprintf(buffer, bufferSize,
    "{\"modulesOnline\":[%d,%d,%d],\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,\"hdop\":%.2f,\"satellites\":%d,\"datetime\":\"%s\",\"resultant_acc\":%.2f,\"resultant_gyro\":%.2f,\"pm10\":%.2f,\"pm25\":%.2f,\"pm100\":%.2f,\"aqi_pm25_us\":%.2f,\"aqi_pm100_us\":%.2f,\"temperature\":%.2f,\"humidity\":%.2f,\"eco2\":%.2f,\"tvoc\":%.2f,\"aqi\":%.2f}",
    // backend will handle modulesOnline and map to exactly what module is online using nodeID
    heartbeat.modulesOnline[0],
    heartbeat.modulesOnline[1],
    heartbeat.modulesOnline[2],
    heartbeat.latitude,
    heartbeat.longitude,
    heartbeat.altitude,
    heartbeat.hdop,
    heartbeat.satellites,
    heartbeat.dateTime,
    heartbeat.resultant_acc,
    heartbeat.resultant_gyro,
    heartbeat.aqi_pm25_us,
    heartbeat.aqi_pm100_us,
    heartbeat.aqi);

  if (len >= bufferSize) return false;
  return true;
}