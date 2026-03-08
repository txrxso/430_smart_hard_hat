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

// 2. Air Quality Alert Values
typedef struct {
  uint8_t seq_num;         // sequence number (0-255, wraps around) for duplicate detection
  uint8_t alert_mask;      // bit 0 = AQI_UBA, bit 1 = PM2.5, bit 2 = PM10
  uint16_t aqi_uba;        // only valid if alert_mask bit 0 set
  uint16_t pm25_aqi;       // only valid if alert_mask bit 1 set
  uint16_t pm100_aqi;      // only valid if alert_mask bit 2 set
} __attribute__((packed)) airQualityAlert_t;

// 3. Noise Heartbeat Values
typedef struct {
  uint16_t noise_db;
  uint16_t reserved[3]; // to make sure 8 bytes in data expected
} __attribute__((packed)) noiseHB_t;

// 4. Noise Alert Values 
typedef struct {
  uint8_t seq_num;         // sequence number (0-255, wraps around) for duplicate detection
  uint8_t reserved;        // padding for alignment
  uint16_t noise_db;
  uint16_t reserved2[2];   // to make sure 8 bytes in data expected
} __attribute__((packed)) noiseAlert_t;


```

## Scenarios on how this node handles CAN communication: 
1. When an ALERT_NOTIFICATION is received from a peripheral module:
   - The gateway node checks the sequence number to filter duplicate retries (see Duplicate Detection below).
   - If not a duplicate, the gateway processes the alert, formats it into an AlertPayload structure, and pushes it to the alertPublishQueue for MQTT publishing.
   - The gateway sends an ALERT_ACK back to the peripheral module to confirm receipt.
2. When a HEARTBEAT_RESPONESE is received from a peripheral module:
   - The gateway node aggregates the heartbeat data from all peripheral modules.
   - This aggregated data is included in the periodic heartbeat messages sent via MQTT.
3. When the gateway node task knows the manualAlertTask() has signaled a manual clear:
   - The gateway formats the corresponding AlertPayload and pushes it to the alertPublishQueue for MQTT publishing.
   - The gateway sends an ALERT_CLEARED message via CAN to all peripheral modules to inform them of the manual alert or clear action so they disregard any exceeding threshold samples for the duration of the cancel timer.

SHOULD NEVER receive the following:
- HEARTBEAT_REQUEST (this is an OUTGOING ONLY message from the gateway to peripheral modules)

## Duplicate Detection

Alert messages include a `seq_num` field (0-255, wraps around) to enable reliable delivery with duplicate filtering:

**Peripheral Node Behavior:**
- Increments `seq_num` only when detecting a NEW alert condition
- Retries use the SAME `seq_num` as the original alert
- Recommended retry interval: 1000ms (gives gateway time to process GPS and send ACK)
- Uses exponential backoff for retries (e.g., 1s → 2s → 4s, capped at 5s)

**Gateway Node Behavior:**
- Tracks last received `seq_num` per peripheral node (indexed by NodeID)
- If `seq_num` matches the last received sequence from that node → duplicate, skip processing
- If `seq_num` differs → new alert, process and update tracker
- Allows peripherals to retry aggressively without flooding MQTT with duplicate messages

**Example Flow:**
```
Peripheral: Alert seq=5 detected → send
Peripheral: No ACK in 1s → retry seq=5 
Peripheral: No ACK in 2s → retry seq=5
Gateway: Receives seq=5 → processes (first time)
Gateway: Receives seq=5 → detects duplicate, ignores
Gateway: Receives seq=5 → detects duplicate, ignores
Gateway: Sends ACK → peripheral stops retrying
Peripheral: New alert seq=6 → send (incremented)
Gateway: Receives seq=6 → processes (new sequence)
```
Thie approach (inspired by TCP), ensures reliable delivery, no duplicate alerts in MQTT, and that new alerts aren't blocked by retry logic.

