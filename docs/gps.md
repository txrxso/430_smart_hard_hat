# GPS Task 

`gpsTask()` manages GPS data acquisition and distribution so that payloads can be aggregated with location and time data. Both periodic baseline sampling and event-triggerd samples/reads are used. 

Supports: 
- Periodic reading of GPS values (every 1.5 seconds)
- Event-driven (using bits and event group) to immediately read the GPS datetime and lat/lon then push it into the queue so the alert/heartbeat payload handling functions can use it
- Periodic saving to NVS (every 5 minutes)
- Computing datetime using ticks elapsed if reading not available, so this uses the last known GPS timestamp

## Types of Readings

**Periodic Reading** 
- maintains baseline
- every 1.5 seconds
- maintains GPS signal fix
- prevents cold start delays
- feeds data for fallback to last known sample and NVS persistence 
- this should run continuously in the background

**Event-triggered Reading** 
- Triggered using `GPS_READ_REQUEST_BIT`
- Provides fresh GPS data with accurate timestamps in UTC when alerts/heartbeats need to aggreated and published
- Timeout _____ ms (TBD)
- Fallback: if fresh read fails (e.g., GPS disconnected), then consumer should use last known data from the `gpsQueue`




## Components

### Event Group Signals 
```c++
extern EventGroupHandle_t gpsEventGroup;
#define GPS_READ_REQUEST_BIT (1 << 0) 
#define GPS_READ_SUCCESS_BIT (1 << 1)
```

Tasks can request a GPS read by setting the bit `GPS_READ_REQUEST_BIT`.
If the bit is set, both request and success bits are cleared, a fresh GPS read is attempted, and if successful, it processes the data and signals back via the `GPS_READ_SUCCESS_BIT`.


### Data Structures
`gpsData` is the primary container for storing our gps data from the readings.
```c++
struct gpsData {
  double latitude;
  double longitude;
  double altitude; // in meters; not reliable

  // parameters below are sent as a quality check
  float hdop; // horizontal dilution of precision (lower means better accuracy)
  int satellites; // number of satellites in view ()
  char dateTime[32];

};
```
`TimeSync` maintains the time synchronization state. 

```c++
struct TimeSync {
    char lastDateTime[20]; // last date/time string from GPS in format "YYYY/MM/DD,HH:MM:SS"
    TickType_t lastSyncTicks;
    bool hasValidSync; // whether we have a valid time sync from GPS
}; // adding this for graceful degradation of GPS time sync in case of signal loss, so we can still have somewhat accurate timestamps for data even without GPS fix

```

## Task Operation 
### Initialization 
1. Always load last known coordinates from NVS 
2. Initialize timesync state to be invalid
3. Set up periodic timers for reads and NVS saves

### Main loop 

1. Check for event-triggered read request. If set, try to get immediate reading -> update queue and set `GPS_READ_SUCCESS_BIT`. Reset periodic timer.
2. Periodic read (if 1.5 seconds elapsed w/o alert or heartbeat need for GPS reading). Update queue with latest data.
3. NVS save only if valid GPS fix available. 


### Enhanced DateTime Feature
Issue: <br>
GPS module may not provide updated timestamp for whatever reason. For contingency, then we still need some timestamping for every alert/heartbeat. Need for graceful degradation if GPS signal loss.

Fix: <br>
Use FreeRTOS tick counter to compute elapsed time since last valid GPS sync.

```c++
void computeDateTime(char* buffer, size_t bufferSize, const char* baseDateTime, TickType_t baseSyncTicks); 
```

1. Parse last known GPS timestamp `baseDateTime`
2. Calculate ticks elapsed since `baseSyncTicks`
3. Convert ticks -> seconds
4. Add elapsed seconds to base timestamp
5. Format results as `"YYYY/MM/DD,HH:MM:SS"`


