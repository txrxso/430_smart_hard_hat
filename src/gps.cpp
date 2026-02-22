#include "gps.h"

// =======================================
// GPS Hardware Globals
// =======================================
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
Preferences gpsPreferences; // to store last known GPS data for fallback when no fix; persists across power cycles

// =======================================
// Low-level GPS functions
// =======================================
void setupGPS() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
}

bool gpsHasFix() {
  return gps.location.isValid() && gps.satellites.value() >= 1;
}

void computeDateTime(char* buffer, size_t bufferSize, const char* baseDateTime,  TickType_t baseSyncTicks) {
  // helper function to compute datetime with millis() offset
  // calculate elapsed ticks since last GPS sync
  TickType_t elapsedTicks = xTaskGetTickCount() - baseSyncTicks;

  // convert to seconds - tick duration = 1/ configTickRate_HZ seconds
  unsigned long elapsedSeconds = elapsedTicks / configTICK_RATE_HZ;

  if (elapsedSeconds == 0) {
    // if no time has elapsed, return base GPS datetime
    strncpy(buffer, baseDateTime, bufferSize - 1);
    buffer[bufferSize - 1] = '\0'; // ensure null termination
    return;
  }

  // parse baseDateTime "YYYY/MM/DD,HH:MM:SS" and add on the time, then format back to string
  int year, month, day, hour, minute, second;
  int parsed = sscanf(baseDateTime, "%d/%d/%d,%d:%d:%d", 
                      &year, &month, &day, &hour, &minute, &second);

  if (parsed == 6) { 
    // add elapsed seconds to the parsed time 
    second += elapsedSeconds;

    // handle time rollover
    minute += second / 60;
    second = second % 60;
    hour += minute / 60;
    minute = minute % 60; 
    day += hour / 24;
    hour = hour % 24;

    // format back to string
    snprintf(buffer, bufferSize, "%04d/%02d/%02d,%02d:%02d:%02d",
             year, month, day, hour, minute, second);
  } 

}

#if MOCK_GPS 
bool gpsRead(gpsData &data) {
  // mock gps, return placeholder values
  data.latitude = 49.2606;
  data.longitude = -123.2460;
  data.altitude = 15.0;
  data.hdop = 0.8;
  data.satellites = 12;

  snprintf(data.dateTime, sizeof(data.dateTime), "2024/01/19,10:30:45");

  return true;

}


#else 
bool gpsRead(gpsData &data) { 
  // update gps data structure with latest values from gps and return true if location updated
  // timeout after 500 ms if no new data 

  unsigned long start = millis();
  bool gotUpdate = false;

  while (true) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // check if valid location data
    if (gps.location.isValid()) {
      // update values
      data.latitude = gps.location.lat();
      data.longitude = gps.location.lng();
      data.altitude = gps.altitude.meters();
      data.hdop = gps.hdop.value() / 100.0;
      data.satellites = gps.satellites.value();
      gotUpdate = true;
    }

    // check if valid time data
    if (gps.date.isValid() && gps.time.isValid()) {
      snprintf(data.dateTime, sizeof(data.dateTime),
                "%04d/%02d/%02d,%02d:%02d:%02d",
                gps.date.year(),
                gps.date.month(),
                gps.date.day(),
                gps.time.hour(),
                gps.time.minute(),
                gps.time.second());
      gotUpdate = true;
    } else {
      snprintf(data.dateTime, sizeof(data.dateTime), "Invalid");
    }

    if (gotUpdate) {
      return true; // exit early if updated
    }

    if (millis() - start > 500) {
      // timeout after 500 ms
      return false;
    }

  } // end of while loop

}

#endif // MOCK_GPS


// =======================================
// GPS Task Manager Implementation
// =======================================
namespace GPSTaskManager {
    void init (State& s, QueueHandle_t gpsQueue, EventGroupHandle_t gpsEventGroup) {
            
        // open NVS storage namespace (read/write)
        ::gpsPreferences.begin("gps_storage",false);

        s.gpsPreferences = &::gpsPreferences; // pointer to module global
        s.gpsQueue = gpsQueue;
        s.gpsEventGroup = gpsEventGroup; 

        // load last known coordinates
        s.lastValidGPS.latitude = ::gpsPreferences.getDouble("latitude", 0.0);
        s.lastValidGPS.longitude = ::gpsPreferences.getDouble("longitude", 0.0);

        // init timers 
        s.lastNVSSave = xTaskGetTickCount(); // init to now so we save to NVS on first run
        s.lastPeriodicRead = xTaskGetTickCount();

        // init timesync state
        s.timesync.hasValidSync = false; // no valid sync yet
        s.timesync.lastDateTime[0] = '\0'; // empty string
        s.timesync.lastSyncTicks = 0;

        #if GPS_DEBUG
        Serial.printf("GPS init. Loaded from NVS: lat=%.6f, lon=%.6f\n", s.lastValidGPS.latitude, s.lastValidGPS.longitude);
        #endif

    } // void init()

    // helper: process fresh GPS data and update state
    static void processData(State& s, const gpsData &newData) {
        // only overwrite coordinates if got valid fix - if no fix, keep last known coordinates (could be from previous run or previous valid reading in this run)
        if (newData.latitude != 0.0 && newData.longitude != 0.0) { 
          // copy all GPS values from fresh reading to the state's last valid cache
          s.lastValidGPS.latitude = newData.latitude;
          s.lastValidGPS.longitude = newData.longitude;
          s.lastValidGPS.altitude = newData.altitude;
          s.lastValidGPS.hdop = newData.hdop;
          s.lastValidGPS.satellites = newData.satellites;
        }
        

        // only update the timesync timestamp if the datetime is VALID 
        // and has changed from the last sync
        if (strcmp(newData.dateTime, "Invalid") != 0 && 
            strcmp(newData.dateTime, s.timesync.lastDateTime) != 0) {
            // update timesync state
            strncpy(s.timesync.lastDateTime, newData.dateTime, sizeof(s.timesync.lastDateTime) - 1);
            s.timesync.lastDateTime[sizeof(s.timesync.lastDateTime) - 1] = '\0'; // ensure null termination
            s.timesync.lastSyncTicks = xTaskGetTickCount();
            s.timesync.hasValidSync = true;

            #if GPS_DEBUG
            Serial.printf("Updated GPS timesync. New datetime: %s\n", s.timesync.lastDateTime);
            #endif
        }

    } // processData()

    // helper: decide what datetime to push to queue 
    static void updateQueue(State& s) {
        // if every synced with GPS time, compute interpolated datetime
        if (s.timesync.hasValidSync) {
            computeDateTime(s.lastValidGPS.dateTime, sizeof(s.lastValidGPS.dateTime), 
                            s.timesync.lastDateTime, s.timesync.lastSyncTicks);
            
        } else {
            // if never synced with GPS, we say no GPS sync available
            snprintf(s.lastValidGPS.dateTime, sizeof(s.lastValidGPS.dateTime), "No GPS Sync");
        }

        // push to queue (overwrites old data)
        xQueueOverwrite(s.gpsQueue, &s.lastValidGPS); // push pointer to last valid GPS data in state to queue for others to read

        #if GPS_DEBUG 
        Serial.printf("GPS queue updated: (%.6f, %.6f) at %s\n", s.lastValidGPS.latitude, s.lastValidGPS.longitude, s.lastValidGPS.dateTime);
        #endif
    } //updateQueue()


    // helper: handle event-triggered read 
    static bool handleEventRead(State& s) {
        EventBits_t bits = xEventGroupGetBits(s.gpsEventGroup);
        if (!(bits & GPS_READ_REQUEST_BIT)) {
            return false; // no read requested
        }

        // mark as received request by clearing bits (reset state)
        xEventGroupClearBits(s.gpsEventGroup, GPS_READ_REQUEST_BIT | GPS_READ_SUCCESS_BIT);

        // event-triggered request received, attempt to read 
        gpsData fresh = {};
        if (gpsRead(fresh)) {
            processData(s, fresh); // update state with fresh data
            updateQueue(s); // push to queue for others to read
            xEventGroupSetBits(s.gpsEventGroup, GPS_READ_SUCCESS_BIT); // mark read success
            return true;
        } 

        #if GPS_DEBUG
        Serial.println("Event-triggered GPS read failed.");
        #endif
        return false; // read failed

    }

    // helper: handle periodic read 
    static void handlePeriodicRead(State& s) {
        gpsData fresh = {};
        if (gpsRead(fresh)) {
            processData(s, fresh); // update state with fresh data
        }
         updateQueue(s); // even if read failed, still update queue with latest cached GPS data and computed datetime (if have synced before)

    }

    // helper: save to NVS
    static void saveToNVS(State& s) {
        if (s.lastValidGPS.latitude == 0.0 && s.lastValidGPS.longitude == 0.0) { 
            #if GPS_DEBUG
            Serial.println("No last valid GPS fix, skipping NVS save.");
            #endif
            return; 
        }

        #if GPS_DEBUG
        TickType_t writeStart = xTaskGetTickCount();
        #endif

        s.gpsPreferences->putDouble("latitude", s.lastValidGPS.latitude);
        s.gpsPreferences->putDouble("longitude", s.lastValidGPS.longitude);

        #if GPS_DEBUG
        TickType_t writeEnd = xTaskGetTickCount();
        unsigned long writeDurationMs = pdTICKS_TO_MS(writeEnd - writeStart);
        Serial.printf("Saved GPS to NVS. Lat=%.6f, Lon=%.6f, took %lu ms\n", 
                      s.lastValidGPS.latitude, s.lastValidGPS.longitude, 
                      writeDurationMs);
        #endif

    }

    // main task loop (never returns) - TODO
    void run(State& s) {
        const TickType_t nvsSaveInterval = pdMS_TO_TICKS(NVS_SAVE_PERIOD * 60 * 1000); // convert minutes to ticks
        const TickType_t periodicReadInterval = pdMS_TO_TICKS(2000); // 2 seconds

        while (true) {
            // 1. Event-triggered read if requested
            bool eventRead = handleEventRead(s);
            if (eventRead) {
                // fresh data - reset periodic timer so we don't immediately do another read after this
                s.lastPeriodicRead = xTaskGetTickCount();
            }


            // 2. Periodic read 
            if (!eventRead && (xTaskGetTickCount() - s.lastPeriodicRead >= periodicReadInterval)) {
                handlePeriodicRead(s);
                s.lastPeriodicRead = xTaskGetTickCount();
            }

            // 3. NVS save 
            if (xTaskGetTickCount() - s.lastNVSSave >= nvsSaveInterval) {
                saveToNVS(s);
                s.lastNVSSave = xTaskGetTickCount();
            }

             // add small delay to avoid tight loop, adjust as needed
             // for responsiveness; note that unless event occurs or periodic read interval elapses, this task is mostly idle waiting for those conditions, so can afford some delay here
             vTaskDelay(pdMS_TO_TICKS(200));

        }

    }
}
