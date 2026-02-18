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
    "{\"event\":\"%s\",\"datetime\":\"%s\",\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,\"noise_db\":%.2f,\"acc\":%.2f,\"gyro\":%.2f}",
    alertTypeToString(alert.event),
    alert.dateTime,
    alert.latitude,
    alert.longitude,
    alert.altitude,
    alert.noise_db,
    alert.resultant_acc,
    alert.resultant_gyro);

  if (len >= bufferSize) return false;

  return true;
  
}

// helper to serialize HeartbeatPayload to string that can be parsed by backend as JSON
bool serializeHB(const HeartbeatPayload& heartbeat, char* buffer, size_t bufferSize) {
  Serial.printf("Serializing Heartbeat Payload: noise_db = %.2f\n", heartbeat.noise_db);

  int len = snprintf(buffer, bufferSize,
    "{\"worker_id\": 10,\"modulesOnline\":[%d,%d],\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f,\"hdop\":%.2f,\"satellites\":%d,\"datetime\":\"%s\",\"resultant_acc\":%.2f,\"resultant_gyro\":%.2f,\"aqi_pm25_us\":%.2f,\"aqi_pm100_us\":%.2f,\"aqi_uba\":%.2f,\"noise_db\":%.2f}",
    // backend will handle modulesOnline and map to exactly what module is online using nodeID
    heartbeat.modulesOnline[0],
    heartbeat.modulesOnline[1],
    heartbeat.latitude,
    heartbeat.longitude,
    heartbeat.altitude,
    (double)heartbeat.hdop,
    heartbeat.satellites,
    heartbeat.dateTime,
    (double)heartbeat.resultant_acc,
    (double)heartbeat.resultant_gyro,
    (double)heartbeat.aqi_pm25_us,
    (double)heartbeat.aqi_pm100_us,
    (double)heartbeat.aqi_uba,
    (double)heartbeat.noise_db);

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
    airQualityHB_t aqData;
    memcpy(&aqData, msg.data, sizeof(aqData));

    collection.payload.aqi_pm25_us = static_cast<float>(aqData.pm25_aqi);
    collection.payload.aqi_pm100_us = static_cast<float>(aqData.pm100_aqi);
    collection.payload.aqi_uba = static_cast<float>(aqData.aqi_uba);
    // mark which node responded 
    collection.payload.modulesOnline[0] = static_cast<u_int8_t>(nodeId);
  }
  else if (nodeId == NODE_NOISE) {
    noiseHB_t noiseData;
    memcpy(&noiseData, msg.data, sizeof(noiseHB_t));
    Serial.printf("Extracted noise_db uint16_t: %d\n", noiseData.noise_db);

    // convert to float for heartbeat payload
    collection.payload.noise_db = static_cast<float>(noiseData.noise_db);
    Serial.printf("Assigned noise_db float: %.2f\n", collection.payload.noise_db);
    Serial.printf("DEBUG: noise_db address = %p, value = %.2f\n", &collection.payload.noise_db, collection.payload.noise_db);
    Serial.printf("DEBUG: hbPayload size = %d bytes\n", sizeof(HeartbeatPayload));
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
    Serial.printf(" Noise: %.2f m\n", alert.noise_db);
    Serial.printf(" Acc: %.2f m\n", alert.resultant_acc);
    Serial.printf(" Gyro: %.2f m\n", alert.resultant_gyro);

}

// ==================================================================
// internal fallback: populate GPS fields from what is only in queue 
// as fallback (not event-driven)
// ==================================================================
static void populateGPSAlertFromQueue(AlertPayload &alert, QueueHandle_t gpsQueue) {
  gpsData latestGpsData; 
  if (xQueuePeek(gpsQueue, &latestGpsData, 0) == pdTRUE) {
    alert.latitude  = latestGpsData.latitude;
    alert.longitude = latestGpsData.longitude;
    alert.altitude  = latestGpsData.altitude;
    strncpy(alert.dateTime, latestGpsData.dateTime, sizeof(alert.dateTime));
  } else {
    alert.latitude  = 0.0;
    alert.longitude = 0.0;
    alert.altitude  = 0.0;
    strncpy(alert.dateTime, "Invalid", sizeof(alert.dateTime));
  }
}

static void populateGPSHBFromQueue(HeartbeatPayload &hb, QueueHandle_t gpsQueue) {
  gpsData latestGpsData;
  if (xQueuePeek(gpsQueue, &latestGpsData, 0) == pdTRUE) {
    hb.latitude   = latestGpsData.latitude;
    hb.longitude  = latestGpsData.longitude;
    hb.altitude   = latestGpsData.altitude;
    hb.hdop       = latestGpsData.hdop;
    hb.satellites = latestGpsData.satellites;
    strncpy(hb.dateTime, latestGpsData.dateTime, sizeof(hb.dateTime) - 1);
    hb.dateTime[sizeof(hb.dateTime) - 1] = '\0';
  } else {
    hb.latitude   = 0.0;
    hb.longitude  = 0.0;
    hb.altitude   = 0.0;
    hb.hdop       = 0.0; // indicate invalid
    hb.satellites = 0;
    strncpy(hb.dateTime, "Invalid", sizeof(hb.dateTime) - 1);
    hb.dateTime[sizeof(hb.dateTime) - 1] = '\0';
  }

}

// ==================================================================
// public helpers to request fresh GPS then attach to alert/heartbeat
// ==================================================================

// populates GPS fields of AlertPayload from latest gpsQueue data
void attachGPSToAlert(AlertPayload &alert, EventGroupHandle_t gpsEventGroup, QueueHandle_t gpsQueue) {
  xEventGroupClearBits(gpsEventGroup, GPS_READ_SUCCESS_BIT);
  xEventGroupSetBits(gpsEventGroup, GPS_READ_REQUEST_BIT); // signal gpsTask to take a reading

  EventBits_t result = xEventGroupWaitBits(
    gpsEventGroup,
    GPS_READ_SUCCESS_BIT,
    pdTRUE, pdFALSE,
    pdMS_TO_TICKS(600)
  );

  if (result & GPS_READ_SUCCESS_BIT) {
    Serial.println("Successfully got fresh GPS data for alert.");
  } else {
    // timed out - no fix, etc. - fallback
    Serial.println("Using fallback data.");
  }
  populateGPSAlertFromQueue(alert, gpsQueue);

}

void attachGPSToHB(HeartbeatPayload &hb, EventGroupHandle_t gpsEventGroup, QueueHandle_t gpsQueue) {
  // are not event-driven - just peek is fine for now
  populateGPSHBFromQueue(hb, gpsQueue);
}