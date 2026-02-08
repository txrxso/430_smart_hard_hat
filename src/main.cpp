/*
Firmware for ESP32 gateway module.
Acts as interface between MQTT broker via Wifi and peripheral modules via CAN bus.
*/

// IMU (MPU6050)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MQTT
#include <WiFi.h>
#include <WiFiClientSecure.h> // for TLS
#include <PubSubClient.h>
#include "wifi_config.h"  // config and credentials
#include "mqtt_config.h"

#include "sensors.h"
#include "data.h"
#include "button.h"
#include "can.h"
#include "debug.h"
#include "certs.h"
#include "data_structs.h"

#include "esp_wifi.h"

// CAN 
#include <driver/twai.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// define DEBUG mode to print stuff
#define ENABLE_TLS 1
#define HEARTBEAT_INTERVAL_MIN 0.5 // set to 5 minutes // in minutes
#define MAX_IMMEDIATE_MQTT_RETRIES 3
#define MQTT_RETRY_DELAY_MS 100

// for maintaining connection state
#if ENABLE_TLS 
WiFiClientSecure wifiClient;
#else 
WiFiClient wifiClient;
#endif 

PubSubClient mqttClient(wifiClient);

TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t canTaskHandle = NULL;
TaskHandle_t mqttPublishTaskHandle = NULL;
TaskHandle_t manualAlertTaskHandle = NULL;
TaskHandle_t heartbeatTaskHandle = NULL; 

QueueHandle_t imuQueue; 
QueueHandle_t gpsQueue; 
QueueHandle_t alertPublishQueue; // MQTT publish alerts
QueueHandle_t heartbeatPublishQueue; // MQTT publish heartbeat data
QueueHandle_t peripheralCanOutgoingQueue; // outgoing CAN messages

EventGroupHandle_t mqttPublishEventGroup = NULL;
EventGroupHandle_t mqttPublishHealthGroup = NULL;

SemaphoreHandle_t hbStateMutex = xSemaphoreCreateMutex(); // global scope

// global heartbeat collection state (shared bewteen tasks)
hbCollection hbCollectState = {
  .isCollecting = false,
}; // start with isCollecting = true so requests can begin immediately

bool connectToMQTT(bool enableTLS = ENABLE_TLS) {
  #if MQTT_DEBUG
  // Serial.println("Attempting MQTT connection...");
  // print debug info
  Serial.print("WiFi status: ");
  Serial.println(isWifiConnected() ? "Connected" : "Disconnected");
  // Serial.print("ESP32 IP: ");
  // Serial.println(WiFi.localIP());
  // Serial.print("Connecting to broker: ");
  // Serial.print(MQTT_SERVER_HIVEMQ_PUBLIC);
  // Serial.print(":");
  // Serial.println(MQTT_PORT_HIVEMQ_PUBLIC);
  #endif 
  bool success = false;

  if (enableTLS) {
    success = mqttClient.connect("ESP32Client", MQTT_USER, MQTT_PSWD);
  }
  else{
    success = mqttClient.connect("ESP32Client");
  }
  
  #if MQTT_DEBUG
  if (success) {
    Serial.println("MQTT connected successfully on first attempt");
  } 
  else {
    Serial.print("MQTT connection failed on first attempt, state=");
    Serial.println(mqttClient.state());
    switch(mqttClient.state()) {
        case -4: Serial.println("MQTT_CONNECTION_TIMEOUT"); break;
        case -3: Serial.println("MQTT_CONNECTION_LOST"); break;
        case -2: Serial.println("MQTT_CONNECT_FAILED"); break;
        case -1: Serial.println("MQTT_DISCONNECTED"); break;
        case 1: Serial.println("MQTT_CONNECT_BAD_PROTOCOL"); break;
        case 2: Serial.println("MQTT_CONNECT_BAD_CLIENT_ID"); break;
        case 3: Serial.println("MQTT_CONNECT_UNAVAILABLE"); break;
        case 4: Serial.println("MQTT_CONNECT_BAD_CREDENTIALS"); break;
        case 5: Serial.println("MQTT_CONNECT_UNAUTHORIZED"); break;
      }
  }
  #endif 
  return success;
}


// TASK: just sends requests for heartbeat data based on timer
// DOES NOT WAIT FOR RESPONSES
void heartbeatRequestTask(void * parameter) {
  #if HEARTBEAT_DEBUG
  Serial.println("Heartbeat Request Task started.");
  #endif
  TickType_t prevWakeTime = xTaskGetTickCount(); // keep track of time to use for next wake up for interval 
  const TickType_t heartbeatInterval = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MIN * 60 * 1000);

  while (true) {
    // protect access to prevent race condition with stuff in incomingcantask()
    if (xSemaphoreTake(hbStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      // only send if not currently collecting 
      if (!hbCollectState.isCollecting) { 
        // only clear and start again if aren't currently mid-collection
        hbCollectState.startTime = xTaskGetTickCount();
        hbCollectState.isCollecting = true; // mark as collecting
        memset(&hbCollectState.payload, 0, sizeof(HeartbeatPayload)); // zero everything

        #if HEARTBEAT_DEBUG
        Serial.println("Sending heartbeat request...");
        #endif

        // send RTR for heartbeat data via CAN
        sendHeartbeatRequest();

      }

      xSemaphoreGive(hbStateMutex); //release
 
    } //semaphore take

  // wait for next interval to request again
  vTaskDelayUntil(&prevWakeTime, heartbeatInterval);

  } // while 
    
} // end of task


// TASK: Incoming CAN Message Monitoring and Handling 
// (centralized handler for all incoming CAN messages, including heartbeat RTR responses)
void incomingCanTask(void * parameter) {
  twai_message_t incoming_msg; 
  twai_message_t outgoing_msg;

  const TickType_t maxFlushDuration = pdMS_TO_TICKS(2000); // max 2 seconds to flush outgoing queue in order 

  static bool flushing=false; 
  static TickType_t startFlushTime = 0;

  while (true) {

    // 1. handle outgoing stuff 
    if (!flushing && uxQueueMessagesWaiting(peripheralCanOutgoingQueue) > 0) {
      flushing = true;
      startFlushTime = xTaskGetTickCount(); // start session timer start

      #if CAN_DEBUG 
      Serial.println("Starting outgoing CAN messages flush session.");
      #endif
    }

    // only if now triggered 'flushing' state, we try to send all outgoing msgs 
    if (flushing) {
      while ((xTaskGetTickCount() - startFlushTime) < maxFlushDuration && xQueueReceive(peripheralCanOutgoingQueue, &outgoing_msg, 0) == pdTRUE) {
        esp_err_t txStatus = twai_transmit(&outgoing_msg, pdMS_TO_TICKS(50));

        if (txStatus == ESP_OK) {
          #if CAN_DEBUG
          Serial.println("Outgoing CAN message transmitted successfully.");
          #endif
        } 
      
        else {
          #if CAN_DEBUG
          Serial.printf("Failed to transmit outgoing CAN message, status=%d\n", txStatus);
          #endif
          // requeue for next iteration
          xQueueSendToFront(peripheralCanOutgoingQueue, &outgoing_msg, pdMS_TO_TICKS(5));
          // flushing = false;
          // break; // stop flushing to avoid blocking - not sure rn if wnat to immediately break after 1st fail
        }
      } 

      // end flush session if time expired or queue empty
      if ((xTaskGetTickCount() - startFlushTime) >= maxFlushDuration || uxQueueMessagesWaiting(peripheralCanOutgoingQueue) == 0) { 
        flushing = false;
        #if CAN_DEBUG
        Serial.println("Ending outgoing CAN flush session.");
        #endif
      }
    }

    // 2. handle incoming stuff 

    // wait for incoming CAN message 
    esp_err_t status = twai_receive(&incoming_msg, pdMS_TO_TICKS(100));

    if (status == ESP_OK) {
      // process incoming message 
      CANMessageType msgType = getMessageTypeFromID(incoming_msg.identifier);
      NodeID nodeId = getNodeIDFromID(incoming_msg.identifier);
      #if CAN_DEBUG
      Serial.printf("Received CAN msg. Type: 0x%02X from Node ID: 0x%02X\n", msgType, nodeId);
      #endif

      // handle the message based on type of CAN message
      if (msgType == HEARTBEAT_RESPONSE && !isCollectionTimedOut(hbCollectState)) {
        if (xSemaphoreTake(hbStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          // check if actually still collecting before writing 
          if (hbCollectState.isCollecting && !isCollectionTimedOut(hbCollectState)) {
            // handle heartbeat response
            aggHeartbeatResponse(nodeId, incoming_msg, hbCollectState); // just aggregates, there is a check later within this task to publish

            # if HEARTBEAT_DEBUG
            Serial.printf("Aggregated heartbeat response from Node ID: 0x%02X -> noise_db: %.2f\n", nodeId, hbCollectState.payload.noise_db);
            #endif
          }

          xSemaphoreGive(hbStateMutex);

        }
      } // if 
      
      else if (msgType == ALERT_NOTIFICATION) {
        // send an ACK back to peripheral module (regardless of cancel active, so can reset state on peripheral)
        twai_message_t ack_msg;
        ack_msg.identifier = buildCANID(CONTROL, ALERT_ACK, GATEWAY_NODE);
        ack_msg.extd = 0;
        ack_msg.rtr = 0;
        ack_msg.data_length_code = 1;
        ack_msg.data[0] = 0x01; // simple ACK payload
        twai_transmit(&ack_msg, pdMS_TO_TICKS(100));

        // check cancel timer 
        if (isCancelActive()) {
          // active, means we just ignore it.
          #if CAN_DEBUG
          Serial.printf("ALERT_NOTIFICATION from Node 0x%02X ignored, cancel timer active.\n", nodeId);
          #endif
        }

        else {
          // declare alert payload 
          AlertPayload alertToSend;
          memset(&alertToSend, 0, sizeof(AlertPayload)); // use memset to clear/avoid garbage values
        
          // parse alert 
          if (nodeId == NODE_NOISE) {
            noiseAlert_t* noiseAlertFrame = (noiseAlert_t* )incoming_msg.data;
            alertToSend.event = NOISE_OVER_THRESHOLD;
            alertToSend.noise_db = static_cast<double>(noiseAlertFrame->noise_db); // alertToSend.noise_db
          } 

          // attach GPS data 
          attachGPSToAlert(alertToSend);

          // forward alert to mqttPublishTask via alertPublishQueue
          if (alertPublishQueue != NULL && xQueueSend(alertPublishQueue, &alertToSend, pdMS_TO_TICKS(5)) == pdTRUE) {
            #if CAN_DEBUG || MQTT_DEBUG
            Serial.printf("Alert from noise node queued.");
            #endif
            
            // signal event group
            xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_CAN_ALERT_BIT);
          }
          

        }
      
      }
  
    } // if ESP_OK = status

    else if (status == ESP_ERR_TIMEOUT) {
      #if CAN_DEBUG
      // Serial.println("No CAN msg recv'd within timeout.");
      #endif 
    }

    else {
      #if CAN_DEBUG 
      Serial.println("Error recv. CAN msg.");
      Serial.println(status);
      #endif

    }


    // finalize the heartbeat collection 
    if (xSemaphoreTake(hbStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) { 
      // check timeout regardless of whether a message was received for a heartbeat response
      // check again if heartbeat collection timed out -> need to signal to mqttPublishEventGroup
      if (hbCollectState.isCollecting && isCollectionTimedOut(hbCollectState)) {
        // 1. Create the frozen snapshot of what we've collected so far
        // Use memcpy instead of assignment to avoid alignment issues with double fields
        HeartbeatPayload snapshot;
        memcpy(&snapshot, &hbCollectState.payload, sizeof(HeartbeatPayload));
        Serial.printf("DEBUG: Right after snapshot copy, noise_db = %.2f\n", snapshot.noise_db);
        
        // 2. Immediately reset global state so Request Task is free to start a new cycle
        hbCollectState.isCollecting = false;

        // 3. Release mutex AFTER we're done with snapshot
        xSemaphoreGive(hbStateMutex);

        // 4. add IMU/GPS data
        imuData temp_imu;
        if (xQueuePeek(imuQueue, &temp_imu, 0) == pdTRUE) {
          snapshot.resultant_acc = temp_imu.resultant_acc;
          snapshot.resultant_gyro = temp_imu.resultant_gyro;
        }
        gpsData temp_gps; 
        if (xQueuePeek(gpsQueue, &temp_gps, 0) == pdTRUE) {
          snapshot.latitude = temp_gps.latitude;
          snapshot.longitude = temp_gps.longitude;
          snapshot.altitude = temp_gps.altitude;
          snapshot.hdop = temp_gps.hdop;
          snapshot.satellites = temp_gps.satellites;
          strncpy(snapshot.dateTime, temp_gps.dateTime, sizeof(snapshot.dateTime) - 1);
          snapshot.dateTime[sizeof(snapshot.dateTime) - 1] = '\0';
        }

        // 5. send to publish queue
        Serial.printf("DEBUG: Before queue send, snapshot.noise_db = %.2f\n", snapshot.noise_db);
        if (xQueueSend(heartbeatPublishQueue, &snapshot, pdMS_TO_TICKS(10)) == pdTRUE) {  // TO DO: add retry in case can't add to QUEUE.
          xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_HEARTBEAT_BIT);
        } 

        Serial.println("Heartbeat collection finalized and queued.");
      }
      else {
        // if didn't enter isTimedOut block, then still release
        xSemaphoreGive(hbStateMutex);
      }

    }


    vTaskDelay(pdMS_TO_TICKS(50)); // shorter for better CAN responsiveness

  }

}

// ===================================================================================

// TASK: MQTT Publishing Task 
// - MQTT publish task is only task that touches Wi-Fi --> do not need a mutex
void mqttPublishTask(void * parameter) {
  TickType_t lastReconnectAttempt = 0;
  const TickType_t reconnectInterval = pdMS_TO_TICKS(2000); // 2 seconds

  // static buffers for JSON payloads
  char alertPayloadBuffer[512];
  char heartbeatPayloadBuffer[512];

  while (true) { 
    // maintain wifi connection 
    if (!isWifiConnected()) {
      WiFi.reconnect();
    }

    // maintain mqtt connection
    if (isWifiConnected()) { 
      if (!mqttClient.connected()) {
        TickType_t now = xTaskGetTickCount();

        if (now - lastReconnectAttempt > reconnectInterval) {
          lastReconnectAttempt = now;
          connectToMQTT();
        }
      }

      else { 
        mqttClient.loop();
      }
    }

    // !!!! ONLY WANT EVENT DRIVEN PUBLISHING TO REDUCE WIFI USAGE AND POWER CONSUMPTION
    EventBits_t bits = xEventGroupWaitBits(
      mqttPublishEventGroup, 
      PUBLISH_IMU_THRESHOLD_BIT | PUBLISH_CAN_ALERT_BIT | 
      PUBLISH_MANUAL_ALERT_BIT | PUBLISH_MANUAL_CLEAR_BIT | 
      PUBLISH_HEARTBEAT_BIT, // bits within event group to wait for 
      pdTRUE,  // clear bits on exit
      pdFALSE, // wait for any bit
      pdMS_TO_TICKS(1000) // timeout after 1 second
    );

    // if any bits are set, then some kind of MQTT publish task is required.  
    if (bits & (PUBLISH_IMU_THRESHOLD_BIT | PUBLISH_CAN_ALERT_BIT | 
                PUBLISH_MANUAL_ALERT_BIT | PUBLISH_MANUAL_CLEAR_BIT)) {
      // prepare payload based on which bit is set 
      // LOGIC: 
      // always try to send and confirm success that message was sent before from alertPublishQueue
      // if success, then try to get next message from queue and send until alertPublishQueue is empty
      // if queue is empty, then no more messages to send, check heartbeatPublishQueue. 
      // if both queues empty, then continue to top of loop (always check for connection)

      AlertPayload alertToSend;

      // process all alerts in queue
      while (alertPublishQueue != NULL && xQueueReceive(alertPublishQueue, &alertToSend, 0) == pdTRUE) {
        // prepare JSON payload
        // (each task that can signal the bit will forward the msg to the queue)
        if (serializeAP(alertToSend, alertPayloadBuffer, sizeof(alertPayloadBuffer))) {
          bool success = false;
          int attempts = 0;

          while (attempts < MAX_IMMEDIATE_MQTT_RETRIES) {
            success =  mqttClient.publish(MQTT_TOPIC_ALERTS, alertPayloadBuffer);

            if (success) {
              #if MQTT_DEBUG
              Serial.println("MQTT alert published successfully.");
              Serial.println(alertPayloadBuffer);
              #endif
              break;
            }

            attempts++;

            #if MQTT_DEBUG
            Serial.printf("MQTT publish failed (Attempt %d/%d). Retrying in %dms...\n", 
                          attempts, MAX_IMMEDIATE_MQTT_RETRIES, MQTT_RETRY_DELAY_MS);
            #endif
            
            // short delay to let the network stack breathe
            vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_DELAY_MS));

          }

          // if publish still failed, put message back at front of queue 
          if (!success) {
            #if MQTT_DEBUG
            Serial.println("All immediate retries failed. Re-queuing to front of queue.");
            #endif
            xQueueSendToFront(alertPublishQueue, &alertToSend, pdMS_TO_TICKS(5));
            break; // exit while loop to retry later 
          } 
          else {
            Serial.println("MQTT alert publish success.");
          }

        }
        else {
          #if MQTT_DEBUG
          Serial.println("Failed to serialize alert payload to JSON.");
          #endif
          continue; // skip to next alert if serialization failed
        }

      }

    }
    

    else if (bits & PUBLISH_HEARTBEAT_BIT) {
      HeartbeatPayload hbToSend;
      
      // process all hbs in queue 
      while (heartbeatPublishQueue != NULL && xQueueReceive(heartbeatPublishQueue, &hbToSend, 0) == pdTRUE) {
        Serial.printf("DEBUG: After queue receive, hbToSend.noise_db = %.2f\n", hbToSend.noise_db);
        // prepare JSON payload
        if (serializeHB(hbToSend, heartbeatPayloadBuffer, sizeof(heartbeatPayloadBuffer))) {
          bool success = mqttClient.publish(MQTT_TOPIC_HEARTBEATS, heartbeatPayloadBuffer);

          // if publish failed, put message back at front of queue 
          if (!success) {
            #if MQTT_DEBUG
            Serial.println("MQTT heartbeat publish failed, re-queuing.");
            Serial.println("Following payload failed to be published:");
            Serial.println(heartbeatPayloadBuffer);
            #endif
            xQueueSendToFront(heartbeatPublishQueue, &hbToSend, pdMS_TO_TICKS(5));
            break; // exit while loop to retry later 
          } 
          else {
            #if MQTT_DEBUG
            Serial.println("MQTT heartbeat published successfully.");
            Serial.println(heartbeatPayloadBuffer);
            #endif
          }

        }
        else {
          #if MQTT_DEBUG
          Serial.println("Failed to serialize heartbeat payload to JSON.");
          #endif
          continue; // skip to next heartbeat if serialization failed
        }
      }


    }
    vTaskDelay(pdMS_TO_TICKS(50)); 

  }

}

// TASK: IMU Monitoring and Sampling
void imuTask(void * parameter) {
  imuData data;

  while (true) {
    // read IMU data and always store most recent value 
    // this value can be used whenever heartbeat is required to be sent.
    readIMU(data);
    if (imuQueue != NULL) { 
      // push into queue for heartbeat to use
      xQueueOverwrite(imuQueue, &data);
    }

    // if fall detected, set event group bit to trigger mqtt publish task
    SafetyEvent event = analyzeIMUData(data);
    if (event != SafetyEvent::NONE) {
      // check if cancel timer is active 
      if (!isCancelActive()) {
        // create alert payload - pull from latest GPS data 
        AlertPayload alertToSend;
        // set alert type
        alertToSend.event = FALL_IMPACT; 
        
        attachGPSToAlert(alertToSend);
        
        // populate IMU measurements using imuData.float resultant_acc and resultant_gyro
        alertToSend.resultant_acc= data.resultant_acc;
        alertToSend.resultant_gyro = data.resultant_gyro;
        alertToSend.noise_db = 0.0; // just set to 0.0 for now

        // TO DO: add severity for SafetyEvent enum 
        
        // send alert to queue 
        if (alertPublishQueue != NULL) {
          BaseType_t result = xQueueSend(alertPublishQueue, &alertToSend, pdMS_TO_TICKS(5));
          #if IMU_DEBUG == 1
          printAlertPayload(alertToSend);
          #endif 
          
          #if IMU_DEBUG == 2 
          Serial.println("Sent IMU reading to queue.");
          if (result == errQUEUE_FULL) {
            Serial.println("Alert queue full.");
          } 
          
          else {
            Serial.println("Alert pushed to publish queue.");
          }
          #endif

        } 
        
        else {
          Serial.println("Alert publish queue is NULL, cannot send alert.");
        }

        // signal MQTT 
        xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_IMU_THRESHOLD_BIT);
      }

      else {
        #if IMU_DEBUG
        Serial.println("IMU event detected but cancel timer active, ignoring.");
        #endif
      }
      
    }

    // sample at 100 Hz (10 ms)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  

}

// TASK: Manual Alert Button Monitoring
void manualAlertTask(void * parameter) {
  while (true) {
    // check for manual alert
    if (isButtonDoublePressed()) { 
      // only signal if cancel timer is not active
      if (!isCancelActive()) {
        // signal MQTT publish for manual alert
        xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_MANUAL_ALERT_BIT);
      }
    }

    // check for manual clear
    if (isButtonHeld()) { 
      // activate cancel timer (ignore alerts for next 2 minutes)
      resetCancelTimer(); 

      // send manual clear alert to outgoing CAN messages to notify other modules
      // (gateway -> peripherals)
      twai_message_t manualClearMsg;
      manualClearMsg.identifier = buildCANID(CONTROL, ALERT_CLEARED, GATEWAY_NODE);
      manualClearMsg.extd = 0;
      manualClearMsg.rtr = 0;
      manualClearMsg.data_length_code = 0; // no payload required

      // push to outgoing queue 
      if (xQueueSend(peripheralCanOutgoingQueue, &manualClearMsg, pdMS_TO_TICKS(5)) == pdTRUE) {
        #if CAN_DEBUG
        Serial.println("Manual clear CAN message queued");
        #endif
      } else {
          #if CAN_DEBUG
          Serial.println("Failed to queue manual clear CAN message");
          #endif
      }
      

      // push to outgoing queue for MQTT (gateway -> dashboard)
      AlertPayload manualClearAlert;
      memset(&manualClearAlert, 0, sizeof(manualClearAlert));
      manualClearAlert.event = MANUAL_CLEAR;
      if (xQueueSend(alertPublishQueue, &manualClearAlert, pdMS_TO_TICKS(5)) == pdTRUE) {
        #if CAN_DEBUG
        Serial.println("Manual clear alert successfully queued for MQTT");
        #endif
      } else {
        #if CAN_DEBUG
        Serial.println("Manual clear alert FAILED TO queue for MQTT");
        #endif
      }


      // signal MQTT publish for manual clear
      xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_MANUAL_CLEAR_BIT);
    }

    // check every 50 ms (task delay)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// TASK: GPS Monitoring and Sampling
void gpsTask(void * parameter) {
  static gpsData lastValidGPSData; // to store last valid GPS data for fallback
  static TimeSync timesync = {
    .lastDateTime = "",
    .lastSyncTicks = 0,
    .hasValidSync = false
  };
  gpsData geodata; 

  while (true) { 
    // read gps data and always store most recent value 
    if (gpsRead(geodata)) { 
      #if GPS_DEBUG 
      if (gpsHasFix()) { 
        Serial.printf("Lat: %.6f\n", geodata.latitude);
        Serial.printf("Lon: %.6f\n", geodata.longitude);
        Serial.printf("Alt: %.2f m\n", geodata.altitude);
        Serial.printf("HDOP: %.2f\n", geodata.hdop);
        Serial.printf("Satellites: %d\n", geodata.satellites);
        Serial.printf("DateTime: %s\n", geodata.dateTime);
      } else { 
        Serial.println(" No valid GPS fix.");
      }
      #endif 

      // always update location first (we decide later if datetime is valid or needs to be modified using ticks)
      lastValidGPSData.latitude = geodata.latitude;
      lastValidGPSData.longitude = geodata.longitude;
      lastValidGPSData.altitude = geodata.altitude;
      lastValidGPSData.hdop = geodata.hdop;
      lastValidGPSData.satellites = geodata.satellites;

      // 1. check if GPS datetime is valid and changed 
      if (strcmp(geodata.dateTime, "Invalid") != 0 && 
          strcmp(geodata.dateTime, timesync.lastDateTime) != 0) { 
      
          // update baseline
          strncpy(timesync.lastDateTime, geodata.dateTime, sizeof(timesync.lastDateTime) - 1);
          timesync.lastDateTime[sizeof(timesync.lastDateTime) - 1] = '\0';
          timesync.lastSyncTicks = xTaskGetTickCount();
          timesync.hasValidSync = true;

          #if GPS_DEBUG
          Serial.printf("GPS sync updated: %s\n", timesync.lastDateTime);
          #endif
          
      }

    }

    // GPS timeout or failure
    else { 
      // no data received within timeout 
      Serial.println("No GPS data received within timeout.");

      // 3. if GPS timeout/failure (no valid data), then use tick offset with last known GPS datetime to estimate current datetime
      // only if at least 1 valid GPS datetime in the past
    }

    // compute enhanced dt 
    if (timesync.hasValidSync) { 
      computeDateTime(lastValidGPSData.dateTime, sizeof(lastValidGPSData.dateTime), 
                      timesync.lastDateTime, timesync.lastSyncTicks);               
    } else {
      // never synced yet
      snprintf(lastValidGPSData.dateTime, sizeof(lastValidGPSData.dateTime), "No GPS Sync");
    }

    // always push to queue 
    xQueueOverwrite(gpsQueue, &lastValidGPSData);
    #if GPS_DEBUG
    Serial.printf("Pushed GPS data to queue. Lat: %.6f, Lon: %.6f, DateTime: %s\n", 
                  lastValidGPSData.latitude, lastValidGPSData.longitude, lastValidGPSData.dateTime);
    #endif

    vTaskDelay(pdMS_TO_TICKS(1500)); // sample every 1.5 seconds

  }

}


// =======================================
// SETUP ON BOOT/RESET 
// =======================================
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  if (!setupIMU()) {
    Serial.println("IMU setup failed. Stopping.");
    while (1) {
      delay(10);
    }
  }
  setupGPS();

  // initialize CAN
  while (!initCAN()) {
    delay(20);
  } 

  mqttPublishEventGroup = xEventGroupCreate();
  mqttPublishHealthGroup = xEventGroupCreate();
  // create queues for sharing data between threads/tasks 
  gpsQueue = xQueueCreate(1, sizeof(gpsData));
  imuQueue = xQueueCreate(1, sizeof(imuData));
  alertPublishQueue = xQueueCreate(20, sizeof(AlertPayload));
  heartbeatPublishQueue = xQueueCreate(10, sizeof(HeartbeatPayload));
  peripheralCanOutgoingQueue = xQueueCreate(10, sizeof(twai_message_t));

  #if MQTT_DEBUG 
  if (mqttPublishEventGroup == NULL || gpsQueue == NULL || imuQueue == NULL) { 
    Serial.println("Failed to create event group or queues.");
    while(1);
  }
  if (mqttPublishHealthGroup == NULL) { 
    Serial.println("Failed to create mqttPublishHealthGroup.");
    while(1);
  }
  #endif 

  // setup wifi as client
  WiFi.mode(WIFI_STA); 
  connectToWifi(); // use this for home network but not for WPA-2 Enterprise
  // connectToWifiEnterprise();

  // configure TLS if enabled
  #if ENABLE_TLS
  wifiClient.setCACert(root_ca);
  #endif

  // configure power management
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

  // increase listen interval 
  wifi_config_t conf;
  esp_err_t ret = esp_wifi_get_config(WIFI_IF_STA, &conf);
  if (ret == ESP_OK) { 
    conf.sta.listen_interval = 10;
    esp_wifi_set_config(WIFI_IF_STA, &conf);
    #if MQTT_DEBUG
    Serial.println("WiFi modem sleep enabled (PS_MAX_MODEM) with listen interval 10");
    #endif
  } else {
    #if MQTT_DEBUG 
    Serial.printf("Failed to get wifi config: %d\n", ret);
    while(1);
    #endif
  }

  // set up mqtt 
  mqttClient.setBufferSize(512);
  #if ENABLE_TLS
  mqttClient.setServer(MQTT_SERVER_HIVEMQ_PRIVATE, MQTT_PORT_HIVEMQ_TLS);
  #else
  mqttClient.setServer(MQTT_SERVER_HIVEMQ_PUBLIC, MQTT_PORT_HIVEMQ_PUBLIC);
  #endif


  connectToMQTT();

  // communication/protocol tasks for core 1 
  xTaskCreatePinnedToCore(
    mqttPublishTask,
    "MQTTPublishTask",
    8192,
    NULL,
    2, // higher priority for alert sending
    &mqttPublishTaskHandle,
    1 // core 0 or 1 
  ); 

  xTaskCreatePinnedToCore(
    incomingCanTask,
    "IncomingCanTask",
    4096,
    NULL,
    2,
    &canTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    heartbeatRequestTask,
    "HeartbeatReqTask",
    2048,
    NULL,
    1,
    &heartbeatTaskHandle,
    1
  );


  // for core 0 

  xTaskCreatePinnedToCore(
    manualAlertTask, 
    "ManualAlertTask",
    2048,
    NULL,
    3,
    &manualAlertTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    imuTask,
    "IMUTask",
    4096,
    NULL,
    2,
    &imuTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    gpsTask,
    "GPSTask",
    4096,
    NULL,
    1,
    &gpsTaskHandle,
    0
  );



  
}

void loop() { 
  // do nothing
}