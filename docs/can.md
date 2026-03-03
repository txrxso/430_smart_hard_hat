# CAN 

## CAN Frame Structure From Peripheral Nodes
Standardized format for gateway to parse. 

```c++
// CAN frame structure for different nodes

// 1. Air Quality Heartbeat Values 
typedef struct {
  uint16_t pm25_aqi;
  uint16_t pm100_aqi;
  uint16_t aqi_uba;
  uint16_t reserved; // to make sure 8 bytes in data expected
} __attribute__((packed)) airQualityHB_t;

// 2. Noise Heartbeat Values
typedef struct {
  uint16_t noise_db;
  uint16_t reserved[3]; // to make sure 8 bytes in data expected
} __attribute__((packed)) noiseHB_t;

// 3. Noise Alert Values 
typedef struct {
  uint16_t noise_db;
  uint16_t reserved[3]; // to make sure 8 bytes in data expected
} __attribute__((packed)) noiseAlert_t;


```

## Scenarios on how this node handles CAN communication: 
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
