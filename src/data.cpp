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
    "{\"worker_id\": 10,\"modulesOnline\":[%d,%d,%d],\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,\"hdop\":%.2f,\"satellites\":%d,\"datetime\":\"%s\",\"resultant_acc\":%.2f,\"resultant_gyro\":%.2f,\"aqi_pm25_us\":%.2f,\"aqi_pm100_us\":%.2f,\"aqi_uba\":%.2f,\"noise_db\":%.2f}",
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
    heartbeat.aqi_uba,
    heartbeat.noise_db);

  if (len >= bufferSize) return false;
  return true;
}


bool isCollectionTimedOut(hbCollection& collection) {
  if (!collection.isCollecting) {
    return false; // not collecting, so cannot be timed out
  }
  bool timedOut = (xTaskGetTickCount() - collection.startTime) > pdMS_TO_TICKS(HEARTBEAT_RESPONSE_TIMEOUT_MS);

  // change state if timed out
  if (timedOut) {
    collection.isCollecting = false;
  }
  return timedOut;
}

void aggHeartbeatResponse(NodeID nodeId, const twai_message_t& msg, hbCollection& collection) {

  // parse CAN data based on node type 
  if (nodeId == NODE_AIR_Q) {
    airQualityHB_t* aqData = (airQualityHB_t*)msg.data;
    collection.payload.aqi_pm25_us = static_cast<float>(aqData->pm25_aqi);
    collection.payload.aqi_pm100_us = static_cast<float>(aqData->pm100_aqi);
    collection.payload.aqi_uba = static_cast<float>(aqData->aqi_uba);
    // mark which node responded 
    collection.payload.modulesOnline[0] = static_cast<u_int8_t>(nodeId);
  }
  else if (nodeId == NODE_NOISE) {
    noiseHB_t* noiseData = (noiseHB_t*)msg.data;
    collection.payload.noise_db = static_cast<float>(noiseData->noise_db);
    // mark which node responded 
    collection.payload.modulesOnline[1] = static_cast<u_int8_t>(nodeId);

  }

}


void printAlertPayload(const AlertPayload& alert) {
    Serial.println("Alert Payload:");
    Serial.printf(" Event: %s\n", alertTypeToString(alert.event));
    Serial.printf(" DateTime: %s\n", alert.dateTime);
    Serial.printf(" Latitude: %.6f\n", alert.latitude);
    Serial.printf(" Longitude: %.6f\n", alert.longitude);
    Serial.printf(" Altitude: %.2f m\n", alert.altitude);
    Serial.println(" Measurements:");
    for (int i = 0; i < 5; i++) {
        if (strlen(alert.measurements[i].key) > 0) {
            Serial.printf("  - %s: %.2f\n", alert.measurements[i].key, alert.measurements[i].value);
        }
    }

}