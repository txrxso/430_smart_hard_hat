#ifndef DATA_H
#define DATA_H

// switch between ESP32 and native build
#ifdef ARDUINO_ARCH_ESP32
  #include <Arduino.h>
#else 
  #include <stdint.h>
  #include <stddef.h>
#endif

/* Scenarios on how this node handles CAN communication: 
1. When an ALERT_NOTIFICATION is received from a peripheral module:
   - The gateway node processes the alert, formats it into an AlertPayload structure, and pushes it to the alertPublishQueue for MQTT publishing.
   - The gateway sends an ALERT_ACK back to the peripheral module to confirm receipt.
2. When a HEARTBEAT_RESPONESE is received from a peripheral module:
   - The gateway node aggregates the heartbeat data from all peripheral modules.
   - This aggregated data is included in the periodic heartbeat messages sent via MQTT.
3. When the gateway node task knows the manualAlertTask() has signaled a manual clear:
   - The gateway formats the corresponding AlertPayload and pushes it to the alertPublishQueue for MQTT publishing.
   - The gateway sends an ALERT_CLEARED message via CAN to all peripheral modules to inform them of the manual alert or clear action so they disregard any exceeding threshold samples for the duration of the cancel timer.

SHOULD NEVER receive the following:
- HEARTBEAT_REQUEST (this is an OUTGOING ONLY message from the gateway to peripheral modules)
*/

// ======================
// IMU and GPS
// ======================

// GPS data 
struct gpsData {
  double latitude;
  double longitude;
  double altitude; // in meters

  // parameters below are sent as a quality check
  float hdop; // horizontal dilution of precision (lower means better accuracy)
  int satellites; // number of satellites in view ()
  char dateTime[32];

};

struct imuData {
  float accX; // in m/s^2
  float accY;
  float accZ;
  float gyroX; // in rad/s
  float gyroY;
  float gyroZ;
  float resultant_acc;
  float resultant_gyro;
};

// ======================
// CAN Message [11 bits : priority, message type, node] [8 bytes data] 
// ======================

// indicates priority for arbitration
enum CANPriority : uint8_t {
    SAFETY_ALERT    = 0, // threshold exceeded, etc.
    CONTROL   = 1, // gateway commands, acks, etc.
    HEARTBEAT = 2, // anything related to heartbeats 
};

// types of messages that can be sent via CAN bus
enum CANMessageType : uint8_t {
  // HIGHEST PRIORITY 0x0- safety critical 
  // alerting lifecycle has two messages per alert: NOTIFICATION and CLEAR
  ALERT_NOTIFICATION = 0x01, // THRESHOLD EXCEEDED, NEEDS TO BE SENT IMMEDIATELY
  ALERT_ACK = 0x02, // GATEWAY ACKNOLWEDGES IT GOT IT
  ALERT_CLEARED = 0x03, // BACK TO NORMAL
  ALERT_CLEARED_ACK = 0x04, // GATEWAY ACKNOWLEDGES CLEAR 
  /* NOTE: 
   acks required, because need ack specifically from gateway. 
   twai filter still acks so if 2 peripheral modules, don't know which one acked it */

  // MEDIUM PRIORITY 0x2 - heartbeat-related (periodic sensor values, "online" modules)
  HEARTBEAT_REQUEST = 0x05,
  HEARTBEAT_RESPONSE = 0x06,

  // LOW PRIORITY 

};

// indicates which node is sending the message
enum NodeID : uint8_t { 
    GATEWAY_NODE = 0x01,
    NODE_NOISE = 0x02,
    NODE_AIR_Q = 0x03,
};

#define THIS_NODE GATEWAY_NODE

// types of alerts that can be sent to mqtt publish task 
// (does not incl. periodic tasks like heartbeats)
enum AlertType { 
    FALL_IMPACT, // IMU
    NOISE_OVER_THRESHOLD, 
    AIR_QUALITY_OVER_THRESHOLD, 
    MANUAL_ALERT, // Button Press (PRESS TWICE QUICKLY WITHIN 1-2 SECONDS)
    MANUAL_CLEAR, // Button Press (HOLD FOR 3 SECONDS)
};

// ======================
// MQTT 
// ======================
struct KVPair {
  char key[16];
  float value;
};

// structure to package alert data for MQTT
struct AlertPayload {
  AlertType event; // TYPE OF ALERT. this is different than SafetyEvent from analyzeImuData
  char dateTime[32];
  // gps data like lat, long, altitude 
  double latitude;
  double longitude;
  double altitude;
  // key value pairs of measured values 
  KVPair measurements[5]; // max 5 measurements for now
};


// CAN frame structure for different nodes
typedef struct {
  uint16_t pm25_aqi;
  uint16_t pm100_aqi;
  uint16_t aqi_uba;
  uint16_t reserved; // to make sure 8 bytes in data expected
} __attribute__((packed)) airQualityHB_t;
typedef struct {
  uint16_t noise_db;
  uint16_t reserved[3]; // to make sure 8 bytes in data expected
} __attribute__((packed)) noiseHB_t;



// stuff for heartbeat payload for MQTT - keep everything as low memory as possible (e.g., double rather than float)
struct hbPayload {
    // array of NodeIDs that responded
    uint8_t modulesOnline[3] = {0}; // (0 = no module)

    // GPS data
    double latitude;
    double longitude;
    double altitude; // in meters

    // parameters below are sent as a quality check
    float hdop; // horizontal dilution of precision (lower means better accuracy)
    int satellites; // number of satellites in view
    char dateTime[32];
    
    // IMU data 
    float resultant_acc; // resultant acceleration
    float resultant_gyro; // resultant gyroscope

    // air quality data from PM
    // float pm10;
    // float pm25;
    // float pm100;
    float aqi_pm25_us;
    float aqi_pm100_us;
    // air quality data from ENS
    // float temperature; 
    // float humidity;
    // float eco2; 
    // float tvoc;
    float aqi;
    // noise data 
    float noise_db; // in decibels
};


// helpers related to data structs
const char* alertTypeToString(AlertType type);
bool serializeAP(const AlertPayload& alert, char* buffer, size_t bufferSize);
bool serializeHB(const hbPayload& heartbeat, char* buffer, size_t bufferSize);


#endif
