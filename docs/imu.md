# IMU Task 

`imuTask()` manages IMU sensor monitoring, fall detection, and continuous alert streaming for worker safety. It samples accelerometer and gyroscope data at 100 Hz, analyzes motion patterns to detect safety events (falls, impacts, freefall), and maintains a continuous alert stream until explicitly cleared by user intervention via the clear button (e.g., hold for >2 seconds).

## Key Features
- **High-frequency sampling** at 100 Hz (10 ms intervals) for responsive safety event detection
- **Multi-threshold fall detection** using acceleration and rotation patterns
- **Continuous alert streaming** - once a fall is detected, alerts are sent continuously with the **original GPS location and timestamp** until the false positive button is pressed
- **State isolation** - only `manualAlertTask()` has authority to change fall active state

## Fall/Impact Detection Algorithm

**Two-Tier Hybrid Approach** for distinguishing real safety events from normal activities (sitting, walking, etc.):

### **Tier 1: Immediate Alert (>8g)**
If resultant acceleration exceeds **8g**, immediately return `SafetyEvent::HEAVY_IMPACT` without validation. Covers:
- Vehicle collisions
- Crushing injuries  
- Falls from great heights
Life-threatening impacts need instant response.

### **Tier 2: State Machine (4-8g)**
For moderate forces, use a **6-state machine** to validate the event pattern and reduce false positives.

#### **States:**

**1. NORMAL** (Starting state)
Three detection paths:

- **Path A: Freefall** (`acc < 0.3g`)  
  → Transition to `FREEFALL`  
  → Stores pre-event orientation for later comparison
  
- **Path B: Direct Impact** (`acc > 4g` AND `jerk > 65 g/s`)  
  → Validates with window buffer (2/3 samples must be high)  
  → Transition to `IMPACT` with event type `DIRECT_IMPACT`
  
- **Path C: Rotational Impact** (`rotation > 500 deg/s` AND `acc > 4g`)  
  → Transition to `IMPACT` with event type `DIRECT_IMPACT`

**2. FREEFALL** (Person is falling)
Waiting for impact or recovery:

- **Impact detected** (`acc > 4g` AND `jerk > 65 g/s`)  
  → If timing valid (100-500ms freefall duration): transition to `IMPACT`  
  → If timing invalid: transition to `RECOVERED` (false alarm)
  
- **Timeout** (>500ms without impact)  
  → Transition to `RECOVERED` (person caught themselves)

**3. IMPACT** (Just experienced impact)
Observation period:

- Wait 1 second for motion to settle  
- Transition to `POST_IMPACT` to check post-impact condition

**4. POST_IMPACT** (Checking recovery vs injury)
Evaluates two criteria:

- **Orientation:** Is person horizontal? (gravity in x-y plane vs z-axis)
- **Motion:** Is person motionless? (`|acc - 1g| < 0.3` AND `gyro < 50 deg/s`)

**Decision logic:**
- If horizontal AND motionless for >2 seconds → `INJURY_LIKELY` (ALERT!)
- If moving → `RECOVERED` (got back up, probably fine)  
- Otherwise → keep observing

**5. INJURY_LIKELY** (Confirmed safety event)
- Continuously return `FALL` or `DIRECT_IMPACT` event
- Stay in this state until manual clear button pressed

**6. RECOVERED** (False alarm)
- Reset state to `NORMAL`
- Resume monitoring

#### Other Notes

**Jerk Calculation** (smoothed over 3 samples with individual time deltas):
- Distinguishes sudden impacts (high jerk) from gradual motions like sitting (low jerk)
- Filters sensor noise while maintaining fast response

**Window Validation**:
- Requires 2/3 samples in buffer to exceed threshold
- Prevents single noise spikes from triggering alerts

**Timing Constraints**:
- Freefall must last 100-500ms 
- Too short = noise, too long = caught themselves
- Post-impact motionless check requires 2 seconds

**Orientation Analysis**:
- Standing: `az ≈ 1g`, `√(ax² + ay²) ≈ 0`
- Lying down: `az ≈ 0`, `√(ax² + ay²) ≈ 1g`

#### **Example Scenarios:**

| **Event** | **State Flow** | **Output** |
|-----------|---------------|------------|
| Fall from ladder | NORMAL → FREEFALL → IMPACT → POST_IMPACT → INJURY_LIKELY | `FALL` alert |
| Struck by falling object | NORMAL → IMPACT → POST_IMPACT → INJURY_LIKELY | `DIRECT_IMPACT` alert |
| Vehicle collision | NORMAL → INJURY_LIKELY (instant) | `HEAVY_IMPACT` alert |
| Sitting down hard, hop | NORMAL (low jerk, filtered) | No alert |
| Tripping but catching self | NORMAL → FREEFALL → RECOVERED | No alert |

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