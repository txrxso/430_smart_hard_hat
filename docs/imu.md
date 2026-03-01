# IMU Task 

`imuTask()` manages IMU sensor monitoring, fall detection, and continuous alert streaming for worker safety. It samples accelerometer and gyroscope data at 100 Hz, analyzes motion patterns to detect safety events (falls, impacts, freefall), and maintains a continuous alert stream until explicitly cleared by user intervention via the clear button (e.g., hold for >2 seconds).

## Key Features
- **High-frequency sampling** at 100 Hz (10 ms intervals) for responsive safety event detection
- **Multi-threshold fall detection** using acceleration and rotation patterns
- **Continuous alert streaming** - once a fall is detected, alerts are sent continuously with the **original GPS location and timestamp** until the false positive button is pressed
- **State isolation** - only `manualAlertTask()` has authority to change fall active state

## Fall/Impact Detection Algorithm
WIP 

### Safety Event Types

```cpp
enum class SafetyEvent { 
  NONE,                      // No safety event detected
  FREEFALL,                  // Sustained < 0.3g acceleration
  MEDIUM_IMPACT,             // 4-10g impact
  HEAVY_IMPACT,              // > 10g impact
  HIGH_ROTATION_AND_ACC      // Combined high rotation + acceleration
};
```

## Data Structures

### IMU Data Container
```cpp
struct imuData {
  float accX, accY, accZ;         // Acceleration in g's
  float gyroX, gyroY, gyroZ;      // Rotation in deg/s
  float resultant_acc;            // Magnitude of acceleration vector
  float resultant_gyro;           // Magnitude of rotation vector
};
```

### Task Manager State
```cpp
namespace IMUTaskManager {
  struct State { 
    // Fall detection persistence
    bool fallActive;                      // True when fall detected, cleared only by button
    AlertPayload originalFallAlert;       // Stored GPS location/timestamp from first detection
    
    // IMU analysis state
    imuData latestIMU;                    // Most recent sensor reading
    float accelWindow[WINDOW_SIZE];       // Circular buffer for acceleration
    float gyroWindow[WINDOW_SIZE];        // Circular buffer for rotation
    int windowIndex;                      // Current position in circular buffers
    
    // Dependencies
    QueueHandle_t imuQueue;               // For heartbeat payload access
    QueueHandle_t gpsQueue;               // For GPS data retrieval
    QueueHandle_t alertPublishQueue;      // For sending alerts to MQTT
    EventGroupHandle_t gpsEventGroup;     // For GPS read requests
    EventGroupHandle_t mqttPublishEventGroup; // For MQTT publish signaling
  };
}
```

## Task Operation

### Initialization

The `IMUTaskManager::init()` function sets up:

1. **Fall state** - initialized to `false` (no active fall)
2. **Circular buffers** - accel initialized to 1.0g (not freefall), gyro to 0.0
3. **Queue handles** - connections to GPS, alerts, and MQTT systems
4. **Event group handles** - for cross-task signaling

### Main Loop (100 Hz)

The `run()` function executes continuously at 10 ms intervals:

```cpp
while (true) {
  // 1. Read sensor
  readIMU(s, data);                        // Sample MPU6050
  xQueueOverwrite(s.imuQueue, &data);      // Update queue for heartbeats
  
  // 2. Analyze for safety events
  SafetyEvent event = analyzeIMUData(s, data);
  
  // 3. Handle new fall detection
  if (event != NONE && !fallActive && !isCancelActive()) {
    - Get GPS location via attachGPSToAlert()
    - Store original alert in originalFallAlert
    - Set fallActive = true
    - Send first alert with fall_detection = 1
  }
  
  // 4. Continue alert stream if fall active
  else if (fallActive && !isCancelActive()) {
    - Use ORIGINAL GPS data from originalFallAlert
    - Send alert with fall_detection = 1
    - Keep same timestamp as first detection
  }
  
  vTaskDelay(10ms);  // 100 Hz sampling
}
```

## Continuous Alert Stream

### Problem
Original implementation sent only **one alert** when fall detected. If the dashboard missed it or needed updates, no mechanism existed to re-notify.

### Solution
Once a fall is detected:

1. **Store original GPS state** - `originalFallAlert` captures lat, lon, altitude, timestamp
2. **Set persistent flag** - `fallActive = true` 
3. **Stream alerts continuously** - Every 10 ms loop iteration sends alert if `fallActive`
4. **Preserve original location** - All subsequent alerts use the **exact GPS coordinates and timestamp** from when fall first occurred

### Why Original GPS Location?

If a worker falls and continues moving (e.g., rolling down a slope, vehicle accident):
- Dashboard needs the **fall location**, not current location
- Original timestamp shows **when** the incident occurred
- Backend can track elapsed time since original event

```cpp
// First detection - capture GPS
attachGPSToAlert(alertToSend, gpsEventGroup, gpsQueue);  // Gets current GPS
setFallDetected(s, alertToSend);  // Stores this GPS + timestamp

// Subsequent alerts - reuse original GPS
getOriginalFallGPSData(s, alertToResend);  // Copies stored lat/lon/time
```

## State Authority (Button Stuff)

### Critical Design Rule
**Only `manualAlertTask()` can change `fallActive` state.**
### Button Clear Sequence
Once a manual clear condition is triggered, the following occurs: 
- `IMUTaskManager::clearFallActive(imuState)` to clear active fall state 
- activate cancel timer to ignore alerts for next 2 minutes 
- send a manual clear alert to outgoing CAN so peripheral nodes also know about alert suppression 
- send MQTT message of type `AlertPayload` to dashboard to indicate alert cleared 

```c++
AlertPayload manualClearAlert;
memset(&manualClearAlert, 0, sizeof(manualClearAlert));
manualClearAlert.event = MANUAL_CLEAR;
manualClearAlert.fall_detection = 0; // explicitly set fall_detection to 0
attachGPSToAlert(manualClearAlert, gpsEventGroup, gpsQueue);

```
This translates to the payload having the following keys and values: 
```json
"event": "MANUAL_CLEAR", 
"fall_detection": 0,
...
```

## Integration with Other Tasks

### GPS Integration
- Uses event-driven GPS reads via `GPS_READ_REQUEST_BIT`
- Calls `attachGPSToAlert()` to get fresh location + timestamp
- GPS datetime in format: `"YYYY/MM/DD,HH:MM:SS"`

### MQTT Integration  
- Signals `PUBLISH_IMU_THRESHOLD_BIT` when alert ready
- Alert includes: event type, GPS coords, timestamp, `fall_detection` flag
- Dashboard field: `fall_detection: 0` or `1` (replaces old `acc`/`gyro` fields)

### Button Integration via Cancel Timer
- `isCancelActive()` checks if in 2-minute false positive window
- When cancel active, fall detection continues but alerts are suppressed
- Allows sensor to keep analyzing without spamming dashboard after false positive

### Heartbeat Integration
- `imuQueue` provides latest IMU data for periodic heartbeat payloads
- Queue updated every 10 ms with most recent sensor reading
- Heartbeat task can peek queue for current acceleration/gyro values

## Hardware Configuration

```cpp
// MPU6050 Settings
mpu.setAccelerometerRange(MPU6050_RANGE_16_G);     // ±16g range
mpu.setGyroRange(MPU6050_RANGE_1000_DEG);          // ±1000 deg/s range  
mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);   // Filter low-freq noise
```

## API Reference

### Public Functions

```cpp
namespace IMUTaskManager {
  // Initialization
  void init(State& s, QueueHandle_t imuQueue, QueueHandle_t gpsQueue, 
            QueueHandle_t alertPublishQueue, EventGroupHandle_t gpsEventGroup,
            EventGroupHandle_t mqttPublishEventGroup);
  
  // Main task loop (never returns)
  void run(State& s);
  
  // State accessors
  bool isFallActive(State& s);           // Check if fall currently active
  void clearFallActive(State& s);        // Clear fall state (button only!)
  imuData getLatestIMU(State& s);        // Get most recent sensor reading
  
  // IMU operations
  void readIMU(State& s, imuData &data);              // Read sensor
  SafetyEvent analyzeIMUData(State& s, const imuData &data);  // Analyze for falls
}
```

### Hardware Function (outside namespace)
```cpp
bool setupIMU();  // Initialize MPU6050 hardware
```
