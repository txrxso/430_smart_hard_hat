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

#include "imu.h"
#include "gps.h"
#include "data.h"
#include "button.h"
#include "can.h"
#include "debug.h"
#include "certs.h"

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
#define HEARTBEAT_INTERVAL_MIN 5  // set to 5 minutes // in minutes
#define MAX_IMMEDIATE_MQTT_RETRIES 3
#define MQTT_RETRY_DELAY_MS 100
#define DUTY_CYCLE 1 // set to 1 to enable tx_duration measurements for duty cycle calculation

#if DUTY_CYCLE
#include <esp_timer.h>

struct DutyCycleStats {
  char event_type;            // 'H'=Heartbeat, 'A'=Alert, 'W'=WiFi, 'M'=MQTT
  uint32_t esp32_timestamp_ms; // millis() when event occurred
  uint32_t duration_us;        // Duration of transmission in microseconds
  bool success;                // Was transmission successful
};

QueueHandle_t dutyCycleQueue = NULL; // queue for sending duty cycle stats from tasks to logger task
#define DC_QUEUE_SIZE 50 

// helper to log without blocking main task execution 
inline void logDutyCycle(char event_type, uint64_t duration_us, bool success) {
  DutyCycleStats stats = {event_type, (uint32_t)millis(), (uint32_t)duration_us, success};
  if (dutyCycleQueue != NULL) {
    xQueueSend(dutyCycleQueue, &stats, 0); // non-blocking send
  }
}

// task to drain queue and print stats to serial
void dutyCycleLogTask(void *parameter) { 
  DutyCycleStats stats;
  while (true) {
    if (xQueueReceive(dutyCycleQueue, &stats, pdMS_TO_TICKS(100)) == pdTRUE) {
      // CSV format: event_type,esp32_timestamp_ms,duration_us,success
      Serial.printf("%c,%lu,%lu,%d\n", stats.event_type, stats.esp32_timestamp_ms, 
                    stats.duration_us, stats.success ? 1 : 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // fast drain
  }
}

#endif

// for maintaining connection state
#if ENABLE_TLS 
WiFiClientSecure wifiClient;
#else 
WiFiClient wifiClient;
#endif 

// track last seq number per node to filter duplicate alerts 
struct { 
  uint8_t lastSeq[2] = {255,255}; // initialize to 255 so first seq of 0 is accepted for all nodes; 2 other nodes
  bool hasReceived[2] = {false, false}; // track if we've received any messages from each node yet
} alertSeqTracker;

GPSTaskManager::State gpsState = {}; // global state for GPS task manager
IMUTaskManager::State imuState = {}; // global state for IMU task manager
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
EventGroupHandle_t gpsEventGroup = NULL;

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
  #endif 
  
  #if DUTY_CYCLE
  uint64_t mqtt_start_us = esp_timer_get_time();
  #endif
  
  bool success = false;

  if (enableTLS) {
    success = mqttClient.connect("ESP32Client", MQTT_USER, MQTT_PSWD);
  }
  else{
    success = mqttClient.connect("ESP32Client");
  }
  
  #if DUTY_CYCLE
  uint64_t mqtt_duration_us = esp_timer_get_time() - mqtt_start_us;
  logDutyCycle('M', mqtt_duration_us, success);
  #endif
  
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

      // ignore messages from our own node
      if (nodeId == THIS_NODE) {
        continue; // skip processing and go to next iteration
      }

      // handle the message based on type of CAN message
      if (msgType == HEARTBEAT_RESPONSE && !isCollectionTimedOut(hbCollectState)) {
        if (xSemaphoreTake(hbStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          // check if actually still collecting before writing 
          if (hbCollectState.isCollecting && !isCollectionTimedOut(hbCollectState)) {
            // handle heartbeat response
            aggHeartbeatResponse(nodeId, incoming_msg, hbCollectState); // just aggregates, there is a check later within this task to publish

            # if HEARTBEAT_DEBUG
            Serial.printf("Aggregated heartbeat response from Node ID: 0x%02X\n", nodeId);
            #endif
          }

          xSemaphoreGive(hbStateMutex);

        }
      } // if 
      
      else if (msgType == ALERT_NOTIFICATION) {
        // send an ACK back to peripheral module (regardless of cancel active, so can reset state on peripheral)
        esp_err_t ackStatus = sendACK();

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
          bool isDuplicate = false; // track if this is a duplicate alert
        
          // parse alert 
          if (nodeId == NODE_NOISE) {
            noiseAlert_t* noiseAlertFrame = (noiseAlert_t* )incoming_msg.data;

            #if CAN_DEBUG
            Serial.printf("[SEQ] NODE_NOISE: received seq=%u, lastSeq=%u, hasReceived=%d\n",
                          noiseAlertFrame->seq_num, alertSeqTracker.lastSeq[0], alertSeqTracker.hasReceived[0]);
            #endif

            // check if duplicate based on seq number - conditions: received it before for the given seq number 
            if (alertSeqTracker.hasReceived[0] && noiseAlertFrame->seq_num == alertSeqTracker.lastSeq[0]) { 
              #if CAN_DEBUG
              Serial.printf("[SEQ] DUPLICATE detected from Node 0x%02X, seq_num: %u - SKIPPING\n", nodeId, noiseAlertFrame->seq_num);  
              #endif
              isDuplicate = true;
            } 
            // not duplicate
            else {
               // new alert - process it 
              alertToSend.event = NOISE_OVER_THRESHOLD;
              alertToSend.noise_db = noiseAlertFrame->noise_db;
              // update seq tracker
              alertSeqTracker.lastSeq[0] = noiseAlertFrame->seq_num;
              alertSeqTracker.hasReceived[0] = true;
              #if CAN_DEBUG
              Serial.printf("[SEQ] NEW alert from Node 0x%02X, seq=%u - PROCESSING\n", nodeId, noiseAlertFrame->seq_num);
              #endif
            }
          }  // node_noise

          if (nodeId == NODE_AIR_Q) {
            airQualityAlert_t* airQualityAlertFrame = (airQualityAlert_t* )incoming_msg.data;

            #if CAN_DEBUG
            Serial.printf("[SEQ] NODE_AIR_Q: received seq=%u, lastSeq=%u, hasReceived=%d\n",
                          airQualityAlertFrame->seq_num, alertSeqTracker.lastSeq[1], alertSeqTracker.hasReceived[1]);
            #endif

            // check if duplicate based on seq number - conditions: received it before for the given seq number 
            if (alertSeqTracker.hasReceived[1] && airQualityAlertFrame->seq_num == alertSeqTracker.lastSeq[1]) { 
              // duplicate - ignore
              #if CAN_DEBUG
              Serial.printf("[SEQ] DUPLICATE detected from Node 0x%02X, seq_num: %u - SKIPPING\n", 
                            nodeId, airQualityAlertFrame->seq_num);
              #endif
              isDuplicate = true;
            } 
            else {
              alertToSend.event = AIR_QUALITY_OVER_THRESHOLD;

              // extract values based on alert mask bits 
              if (airQualityAlertFrame->alert_mask & 0x01) {
                alertToSend.aqi_uba = airQualityAlertFrame->aqi_uba;
              }
              if (airQualityAlertFrame->alert_mask & 0x02) {
                alertToSend.aqi_pm25_us = airQualityAlertFrame->pm25_aqi;
              }
              if (airQualityAlertFrame->alert_mask & 0x04) {
                alertToSend.aqi_pm100_us = airQualityAlertFrame->pm100_aqi;
              }

              //update seq tracker
              alertSeqTracker.lastSeq[1] = airQualityAlertFrame->seq_num;
              alertSeqTracker.hasReceived[1] = true;
              #if CAN_DEBUG
              Serial.printf("[SEQ] NEW alert from Node 0x%02X, seq=%u - PROCESSING\n", nodeId, airQualityAlertFrame->seq_num);
              #endif
            }
          } // node_air_q

          // Only process alert if not a duplicate
          if (!isDuplicate) {
            attachGPSToAlert(alertToSend, gpsEventGroup, gpsQueue); 

            // forward alert to mqttPublishTask via alertPublishQueue
            if (alertPublishQueue != NULL && xQueueSend(alertPublishQueue, &alertToSend, pdMS_TO_TICKS(5)) == pdTRUE) {
              #if CAN_DEBUG == 2 || MQTT_DEBUG
              Serial.printf("Alert queued.");
              #endif
            
              // signal event group
              xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_CAN_ALERT_BIT);
            }
          } else {
            #if CAN_DEBUG
            Serial.println("[SEQ] Duplicate alert NOT forwarded to MQTT");
            #endif
          }

        }
      
      }
  
    } // if ESP_OK = status

    else if (status == ESP_ERR_TIMEOUT) {
      // no message received within timeout window, can use this block to check for heartbeat collection timeout if currently collecting
      #if CAN_DEBUG == 2
      Serial.println("No CAN message received within timeout.");
      #endif

    }

    #if CAN_DEBUG 
    else {
      Serial.println("Error recv. CAN msg.");
      Serial.println(status);
    }
    #endif


    // finalize the heartbeat collection 
    if (xSemaphoreTake(hbStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) { 
      // check timeout regardless of whether a message was received for a heartbeat response
      // check again if heartbeat collection timed out -> need to signal to mqttPublishEventGroup
      if (hbCollectState.isCollecting && isCollectionTimedOut(hbCollectState)) {
        // 1. Create the frozen snapshot of what we've collected so far
        // Use memcpy instead of assignment to avoid alignment issues with double fields
        HeartbeatPayload snapshot;
        memcpy(&snapshot, &hbCollectState.payload, sizeof(HeartbeatPayload));
        
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

        // this will just peek latest GPS data for now since are not event-driven 
        // - TO DO: make event-driven in future for better accuracy
        attachGPSToHB(snapshot, gpsEventGroup, gpsQueue); 
        
        // 5. send to publish queue
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
      #if DUTY_CYCLE
      uint64_t wifi_start_us = esp_timer_get_time();
      #endif
      
      WiFi.reconnect();
      
      #if DUTY_CYCLE
      uint64_t wifi_duration_us = esp_timer_get_time() - wifi_start_us;
      logDutyCycle('W', wifi_duration_us, isWifiConnected());
      #endif
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
            #if DUTY_CYCLE
            uint64_t tx_start_us = esp_timer_get_time();
            #endif
            
            success =  mqttClient.publish(MQTT_TOPIC_ALERTS, alertPayloadBuffer);
            
            #if DUTY_CYCLE
            uint64_t tx_duration_us = esp_timer_get_time() - tx_start_us;
            logDutyCycle('A', tx_duration_us, success);
            #endif

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
        // prepare JSON payload
        if (serializeHB(hbToSend, heartbeatPayloadBuffer, sizeof(heartbeatPayloadBuffer))) {
          #if DUTY_CYCLE
          uint64_t tx_start_us = esp_timer_get_time();
          #endif
          
          bool success = mqttClient.publish(MQTT_TOPIC_HEARTBEATS, heartbeatPayloadBuffer);
          
          #if DUTY_CYCLE
          uint64_t tx_duration_us = esp_timer_get_time() - tx_start_us;
          logDutyCycle('H', tx_duration_us, success);
          #endif

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

// TASK: Manual Alert Button Monitoring
void manualAlertTask(void * parameter) {
  #if BUTTON_DEBUG
  static unsigned long lastDebugPrint = 0;
  static unsigned long lastISRCount = 0;
  #endif
  
  while (true) {
    #if BUTTON_DEBUG
    // Print debug info every 10 seconds (reduce serial overhead)
    if (millis() - lastDebugPrint > 10000) {
      unsigned long currentISRCount = getButtonISRCount();
      Serial.print("[BUTTON] ISR count: ");
      Serial.print(currentISRCount);
      Serial.print(" (delta: ");
      Serial.print(currentISRCount - lastISRCount);
      Serial.print("), Pin state: ");
      Serial.print(digitalRead(BUTTON_PIN) ? "HIGH" : "LOW");
      Serial.print(", Button pressed: ");
      Serial.println(buttonState() ? "YES" : "NO");
      lastDebugPrint = millis();
      lastISRCount = currentISRCount;
    }
    #endif
    
    // check for manual alert
    if (isButtonDoublePressed()) { 
      #if BUTTON_DEBUG
      Serial.println("[MAIN] Double-press detected - triggering manual alert");
      #endif
      
      // only signal if cancel timer is not active
      if (!isCancelActive()) {
        // Create and queue manual alert payload
        AlertPayload manualAlertPayload;
        memset(&manualAlertPayload, 0, sizeof(manualAlertPayload));
        manualAlertPayload.event = MANUAL_ALERT;
        attachGPSToAlert(manualAlertPayload, gpsEventGroup, gpsQueue);
        
        if (xQueueSend(alertPublishQueue, &manualAlertPayload, pdMS_TO_TICKS(5)) == pdTRUE) {
          // signal MQTT publish for manual alert
          xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_MANUAL_ALERT_BIT);
          
          #if BUTTON_DEBUG
          Serial.println("[MAIN] Manual alert queued for MQTT publish");
          #endif
        } else {
          #if BUTTON_DEBUG
          Serial.println("[MAIN] Failed to queue manual alert!");
          #endif
        }
      } else {
        #if BUTTON_DEBUG
        Serial.println("[MAIN] Manual alert BLOCKED - cancel timer active");
        #endif
      }
    }

    // check for manual clear
    if (isButtonHeld()) {
      #if BUTTON_DEBUG
      Serial.println("[MAIN] Button hold detected - triggering manual clear");
      #endif 
      // Clear fall detection state if active
      if (IMUTaskManager::isFallActive(imuState)) {
        IMUTaskManager::clearFallActive(imuState);
        #if IMU_DEBUG
        Serial.println("Fall cleared via button - sending fall_detection=0");
        #endif
      }
      
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
      manualClearAlert.fall_detection = 0; // explicitly set fall_detection to 0
      attachGPSToAlert(manualClearAlert, gpsEventGroup, gpsQueue);
      
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

// TASK: IMU Monitoring and Sampling
void imuTask(void * parameter) {
  IMUTaskManager::State* state = (IMUTaskManager::State*)parameter;
  IMUTaskManager::init(*state, imuQueue, gpsQueue, alertPublishQueue, mqttPublishEventGroup, gpsEventGroup);
  IMUTaskManager::run(*state);
}

// TASK: GPS Monitoring and Sampling
void gpsTask(void * parameter) {
    GPSTaskManager::State* state = (GPSTaskManager::State*)parameter;
    GPSTaskManager::init(*state, gpsQueue, gpsEventGroup);
    GPSTaskManager::run(*state);
}


// =======================================
// SETUP ON BOOT/RESET 
// =======================================
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  if (!setupIMU()) {
    while (1) {
      Serial.println("IMU setup failed. Stopping.");
      delay(10);
    }
  }

  // hardware setup
  setupGPS();
  buttonInit();

  // initialize CAN
  while (!initCAN()) {
    delay(20);
  } 

  #if DUTY_CYCLE
  dutyCycleQueue = xQueueCreate(DC_QUEUE_SIZE, sizeof(DutyCycleStats));
  xTaskCreatePinnedToCore(
    dutyCycleLogTask,
    "DutyCycleLogTask",
    2048,
    NULL,
    0,  // lowest priority
    NULL,
    1   // pin to core 1
  );
  Serial.println("DUTY_CYCLE_START"); // marker for Python script
  #endif

  mqttPublishEventGroup = xEventGroupCreate();
  mqttPublishHealthGroup = xEventGroupCreate();
  gpsEventGroup = xEventGroupCreate();
  // create queues for sharing data between threads/tasks 
  gpsQueue = xQueueCreate(1, sizeof(gpsData));
  imuQueue = xQueueCreate(1, sizeof(imuData));
  alertPublishQueue = xQueueCreate(20, sizeof(AlertPayload));
  heartbeatPublishQueue = xQueueCreate(10, sizeof(HeartbeatPayload));
  peripheralCanOutgoingQueue = xQueueCreate(10, sizeof(twai_message_t));

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
  } else {
    while(1);
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
    2, 
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
    &imuState, // pass pointer to IMU task manager state
    2,
    &imuTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    gpsTask,
    "GPSTask",
    4096,
    &gpsState, // pass pointer to GPS task manager state
    1,
    &gpsTaskHandle,
    0
  );



  
}

void loop() { 
  // do nothing
}