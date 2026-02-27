#  Firmware State Machine Diagrams

Gateway node firmware architecture.

Ensure that markdown preview has mermaid support (extension in VS code).

## 1. System Architecture Overview

### 1.1 Hardware Interfaces
1. IMU (MPU6050) : I2C
2. GPS (NEO 6M): UART
3. CAN Transceiver: TWAI
4. Manual Alert Button 

### 1.2 FreeRTOS Tasks

| Task Name | Function | Priority | Core | Stack Size | Description |
|-----------|----------|----------|------|------------|-------------|
| `mqttPublishTask` | `mqttPublishTask()` | 2 | 1 | 8192 | Highest priority - publishes alerts and heartbeats to MQTT broker |
| `incomingCanTask` | `incomingCanTask()` | 2 | 1 | 4096 | Handles incoming/outgoing CAN messages, aggregates heartbeat responses |
| `heartbeatRequestTask` | `heartbeatRequestTask()` | 1 | 1 | 2048 | Periodic timer - sends heartbeat RTR requests on CAN bus. Lowest priority. |
| `manualAlertTask` | `manualAlertTask()` | 3 | 0 | 2048 | Monitors button press, generates manual alerts. Highest priority because of false negatives. |
| `imuTask` | `imuTask()` | 2 | 0 | 4096 | Reads IMU data, queues for alerts/heartbeats. Fall/impact detection. |
| `gpsTask` | `gpsTask()` | 1 | 0 | 4096 | Parses GPS NMEA sentences, queues position/datetime. Lowest priority because less critical compared to alerting. |

### 1.3 Queues

| Queue Name | Type | Purpose |
|------------|------|---------|
| `imuQueue` | `imuData` | Passes IMU readings (acceleration, gyroscope) to MQTT publish task |
| `gpsQueue` | `gpsData` | Passes GPS data (lat, lon, altitude, datetime) to MQTT publish task |
| `alertPublishQueue` | `AlertPayload` | Queues alerts (manual, CAN, IMU threshold) for MQTT publishing |
| `heartbeatPublishQueue` | `HeartbeatPayload` | Queues periodic heartbeat data for MQTT publishing |
| `peripheralCanOutgoingQueue` | `twai_message_t` | Queues outgoing CAN messages (ACKs, commands) |

### 1.4 Event Groups

| Event Group | Bits | Purpose |
|-------------|------|---------|
| `mqttPublishEventGroup` | `PUBLISH_IMU_THRESHOLD_BIT` (1 << 0)<br/>`PUBLISH_CAN_ALERT_BIT` (1 << 1)<br/>`PUBLISH_MANUAL_ALERT_BIT` (1 << 2)<br/>`PUBLISH_MANUAL_CLEAR_BIT` (1 << 3)<br/>`PUBLISH_HEARTBEAT_BIT` (1 << 4) | Signals MQTT publish task when data is ready to send |
| `gpsEventGroup` | `GPS_READ_REQUEST_BIT` (1 << 0)<br/>`GPS_READ_SUCCESS_BIT` (1 << 1) | Coordinates GPS reading state and signals valid fix |

## 2. State Machine Diagrams 

TO DO