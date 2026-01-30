/*
Firmware for ESP32 gateway module.
Acts as interface between MQTT broker via Wifi and peripheral modules via CAN bus.

TO DO: 
- why is alert payload being published for values of 2.79 gyro and 1.23 acc (resultant)
- change mqtt heartbeat to publish "modulesOnline":[0,0,0] to [1,0,0] if there is an IMU reading
*/

// IMU (MPU6050)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_config.h"  // config and credentials
#include "mqtt_config.h"

#include "sensors.h"
#include "data.h"
#include "button.h"
#include "can.h"
#include "debug.h"

#include "esp_wifi.h"

// CAN 
#include <driver/twai.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// define DEBUG mode to print stuff
#define HEARTBEAT_INTERVAL_MIN 0.5 // set to 5 minutes // in minutes

// for maintaining connection state
WiFiClient wifiClient;
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
QueueHandle_t canTxQueue; // outgoing CAN messages

EventGroupHandle_t mqttPublishEventGroup = NULL;
EventGroupHandle_t mqttPublishHealthGroup = NULL;

// global heartbeat collection state (shared bewteen tasks)
hbCollection hbCollectState = {
  .isCollecting = false,
}; // start with isCollecting = true so requests can begin immediately

bool connectToMQTT() {
  #if MQTT_DEBUG
  Serial.println("Attempting MQTT connection...");
  // print debug info
  Serial.print("WiFi status: ");
  Serial.println(isWifiConnected() ? "Connected" : "Disconnected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connecting to broker: ");
  Serial.print(MQTT_SERVER_HIVEMQ_PUBLIC);
  Serial.print(":");
  Serial.println(MQTT_PORT_HIVEMQ_PUBLIC);
  #endif 

  bool success = mqttClient.connect("ESP32Client");

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
  uint32_t hbReqCount = 0; // for debugging purposes

  while (true) {
    // only send if not currently collecting 
    if (!hbCollectState.isCollecting) { 
      // initialize hbCollection state
      hbCollectState.startTime = xTaskGetTickCount();
      hbCollectState.isCollecting = true; // mark as collecting
      memset(&hbCollectState.payload, 0, sizeof(hbPayload)); // zero everything

      #if HEARTBEAT_DEBUG
      Serial.println("Sending heartbeat request...");
      #endif

      // send RTR for heartbeat data via CAN
      sendHeartbeatRequest();

      hbReqCount++;

    }

    else {
      #if HEARTBEAT_DEBUG
      Serial.println("Skipping heartbeat request.");
      #endif
    }

    // wait for next interval to request again
    vTaskDelayUntil(&prevWakeTime, heartbeatInterval);

  }

}



// TASK: Incoming CAN Message Monitoring and Handling 
// (centralized handler for all incoming CAN messages, including heartbeat RTR responses)
void incomingCanTask(void * parameter) {
  twai_message_t incoming_msg; 

  while (true) {
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
        // handle heartbeat response
        aggHeartbeatResponse(nodeId, incoming_msg, hbCollectState);

        # if HEARTBEAT_DEBUG
        Serial.printf("Aggregated heartbeat response from Node ID: 0x%02X\n", nodeId);
        #endif
        
      }
      else if (msgType == ALERT_NOTIFICATION) {
        // send an ACK back to peripheral module
        twai_message_t ack_msg;
        ack_msg.identifier = buildCANID(CONTROL, ALERT_ACK, GATEWAY_NODE);
        ack_msg.extd = 0;
        ack_msg.rtr = 0;
        ack_msg.data_length_code = 1;
        ack_msg.data[0] = 0x01; // simple ACK payload
        twai_transmit(&ack_msg, pdMS_TO_TICKS(100));

        // handle alert notification 
        
        // parse CAN message

        // forward alert to mqttPublishTask via alertPublishQueue
        // signal to event group
        

      }
      else if (msgType == ALERT_CLEARED) {
        // handle alert cleared 
        // TO DO
      }

  
    }

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

    // check timeout regardless of whether a message was received for a heartbeat response
    // check again if heartbeat collection timed out -> need to signal to mqttPublishEventGroup
    if (hbCollectState.isCollecting && isCollectionTimedOut(hbCollectState)) {
      #if MQTT_DEBUG || HEARTBEAT_DEBUG
      Serial.println("Heartbeat collection timed out, signaling MQTT publish task.");
      #endif

      // add local IMU and gps data
      imuData temp_imu;
      if (xQueuePeek(imuQueue, &temp_imu, 0) == pdTRUE) {
        hbCollectState.payload.resultant_acc = temp_imu.resultant_acc;
        hbCollectState.payload.resultant_gyro = temp_imu.resultant_gyro;
      }
      gpsData temp_gps; 
      if (xQueuePeek(gpsQueue, &temp_gps, 0) == pdTRUE) {
        hbCollectState.payload.latitude = temp_gps.latitude;
        hbCollectState.payload.longitude = temp_gps.longitude;
        hbCollectState.payload.altitude = temp_gps.altitude;
        hbCollectState.payload.hdop = temp_gps.hdop;
        hbCollectState.payload.satellites = temp_gps.satellites;
        strncpy(hbCollectState.payload.dateTime, temp_gps.dateTime, sizeof(hbCollectState.payload.dateTime) - 1);
        hbCollectState.payload.dateTime[sizeof(hbCollectState.payload.dateTime) - 1] = '\0';
      }


      // now publish
      xQueueSend(heartbeatPublishQueue, &hbCollectState.payload, pdMS_TO_TICKS(10));
      xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_HEARTBEAT_BIT);
      hbCollectState.isCollecting = false; // reset collection state flag
      Serial.println("Heartbeat collection timed out. Resetting flag.");
    }


    vTaskDelay(pdMS_TO_TICKS(100)); // small delay to yield CPU

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
          bool success = mqttClient.publish(MQTT_TOPIC_ALERTS, alertPayloadBuffer);

          // if publish failed, put message back at front of queue 
          if (!success) {
            #if MQTT_DEBUG
            Serial.println("MQTT publish failed, re-queuing alert.");
            Serial.println("Following payload failed to be published:");
            Serial.println(alertPayloadBuffer);
            #endif
            xQueueSendToFront(alertPublishQueue, &alertToSend, pdMS_TO_TICKS(5));
            break; // exit while loop to retry later 
          } 
          else {
            #if MQTT_DEBUG
            Serial.println("MQTT alert published successfully.");
            Serial.println(alertPayloadBuffer);
            #endif
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
      hbPayload hbToSend;
      
      // process all hbs in queue 
      while (heartbeatPublishQueue != NULL && xQueueReceive(heartbeatPublishQueue, &hbToSend, 0) == pdTRUE) {
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
        
        // use xQueuePeek to not remove data from queue
        gpsData latestGpsData;
        if (xQueuePeek(gpsQueue, &latestGpsData, 0) == pdTRUE) {
          // get lat, long, altitude, dateTime from gpsQueue 
          alertToSend.latitude = latestGpsData.latitude;
          alertToSend.longitude = latestGpsData.longitude;
          alertToSend.altitude = latestGpsData.altitude;
          strncpy(alertToSend.dateTime, latestGpsData.dateTime, sizeof(alertToSend.dateTime));
        } 
        else {
          alertToSend.latitude = 0.0;
          alertToSend.longitude = 0.0;
          alertToSend.altitude = 0.0;
          strncpy(alertToSend.dateTime, "Invalid", sizeof(alertToSend.dateTime));
        }
        
        // populate IMU measurements using imuData.float resultant_acc and resultant_gyro
        strncpy(alertToSend.measurements[0].key, "acc", sizeof(alertToSend.measurements[0].key));
        alertToSend.measurements[0].value = data.resultant_acc;

        strncpy(alertToSend.measurements[1].key, "gyro", sizeof(alertToSend.measurements[1].key));
        alertToSend.measurements[1].value = data.resultant_gyro;

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

      // TO DO: send manual clear alert to outgoing CAN messages to notify other modules

      // signal MQTT publish for manual clear
      xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_MANUAL_CLEAR_BIT);
    }

    // check every 50 ms (task delay)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// TASK: GPS Monitoring and Sampling
void gpsTask(void * parameter) {
  gpsData geodata; 

  while (true) { 
    // read gps data and always store most recent value 
    if (gpsRead(geodata)) { 
      #if GPS_DEBUG 
      Serial.print("Read GPS data.");

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

      // push data to queue for other tasks to use
      xQueueOverwrite(gpsQueue, &geodata);

      #if GPS_DEBUG
      Serial.println("GPS data pushed to queue.");
      #endif

    }

    else { 
      // no data received within timeout 
      Serial.println("No GPS data received within timeout.");
    }

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
  alertPublishQueue = xQueueCreate(10, sizeof(AlertPayload));
  heartbeatPublishQueue = xQueueCreate(10, sizeof(AlertPayload));
  canTxQueue = xQueueCreate(10, sizeof(twai_message_t));

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

  setupCAN();

  // setup wifi as client
  WiFi.mode(WIFI_STA); // TO DO: sleep mode? low power consumption. can be interrupt driven?
  connectToWifi(); // use this for home network but not for WPA-2 Enterprise
  // connectToWifiEnterprise();

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
  mqttClient.setServer(MQTT_SERVER_HIVEMQ_PUBLIC, MQTT_PORT_HIVEMQ_PUBLIC);

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