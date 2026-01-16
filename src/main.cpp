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

// CAN 
#include <driver/twai.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// define DEBUG mode to print stuff
#define DEBUG_MODE 1 
#define HEARTBEAT_INTERVAL_MIN 5 // in minutes

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

bool connectToMQTT() {
  #if DEBUG_MODE
  Serial.println("Attempting MQTT connection...");
  // print debug info
  Serial.print("WiFi status: ");
  Serial.println(isWifiConnected() ? "Connected" : "Disconnected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connecting to broker: ");
  Serial.print(MQTT_SERVER_PUBLIC_MOSQUITTO);
  Serial.print(":");
  Serial.println(MQTT_PORT_MOSQUITTO);
  #endif 

  bool success = mqttClient.connect("ESP32Client");

  #if DEBUG_MODE
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
  TickType_t prevWakeTime = xTaskGetTickCount(); // keep track of time to use for next wake up for interval 
  const TickType_t heartbeatInterval = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MIN * 60 * 1000);

  while (true) {
    // send RTR for heartbeat data via CAN
    sendHeartbeatRequest();

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
      #if CAN_DEBUG
      CANMessageType msgType = getMessageTypeFromID(incoming_msg.identifier);
      NodeID nodeId = getNodeIDFromID(incoming_msg.identifier);
      Serial.printf("Received CAN msg. Type: 0x%02X from Node ID: 0x%02X\n", msgType, nodeId);
      #endif

      // handle the message based on type of CAN message
      switch (incoming_msg.identifier) {
        case ALERT_NOTIFICATION:
          // parse CAN data and populate AlertPayload and send to alertPublishQueue for MQTT task 
          break;

        case HEARTBEAT_RESPONSE: 
          break; 

        case ALERT_CLEARED_ACK:
          break;

      }


    }

    else if (status == ESP_ERR_TIMEOUT) {
      #if CAN_DEBUG
      Serial.println("No CAN msg recv'd within timeout.");
      #endif 
    }

    else {
      #if CAN_DEBUG 
      Serial.println("Error recv. CAN msg.");
      Serial.println(status);
      #endif
    }

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
          bool success = mqttClient.publish(MQTT_TOPIC_TEST, alertPayloadBuffer);

          // if publish failed, put message back at front of queue 
          if (!success) {
            #if DEBUG_MODE
            Serial.println("MQTT publish failed, re-queuing alert.");
            Serial.println("Following payload failed to be published:");
            Serial.println(alertPayloadBuffer);
            #endif
            xQueueSendToFront(alertPublishQueue, &alertToSend, pdMS_TO_TICKS(5));
            break; // exit while loop to retry later 
          } 
          else {
            #if DEBUG_MODE
            Serial.println("MQTT alert published successfully.");
            Serial.println(alertPayloadBuffer);
            #endif
          }

        }
        else {
          #if DEBUG_MODE
          Serial.println("Failed to serialize alert payload to JSON.");
          #endif
          continue; // skip to next alert if serialization failed
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
          #if DEBUG_MODE
          if (result == errQUEUE_FULL) {
            Serial.println("Alert queue full.");
          } else {
            Serial.println("Alert pushed to publish queue.");
          }
          #endif 
        } else {
          #if DEBUG_MODE
          Serial.println("Alert publish queue is NULL, cannot send alert.");
          #endif
        }

        // signal MQTT 
        xEventGroupSetBits(mqttPublishEventGroup, PUBLISH_IMU_THRESHOLD_BIT);
      }

      else {
        #if DEBUG_MODE
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
      #if DEBUG_MODE 
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

      #if DEBUG_MODE
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
  // create queues for sharing data between threads/tasks 
  gpsQueue = xQueueCreate(1, sizeof(gpsData));
  imuQueue = xQueueCreate(1, sizeof(imuData));
  alertPublishQueue = xQueueCreate(10, sizeof(AlertPayload));
  heartbeatPublishQueue = xQueueCreate(10, sizeof(AlertPayload));
  canTxQueue = xQueueCreate(10, sizeof(twai_message_t));

  #if DEBUG_MODE 
  if (mqttPublishEventGroup == NULL || gpsQueue == NULL || imuQueue == NULL) { 
    Serial.println("Failed to create event group or queues.");
    while(1);
  }
  #endif 

  // setup wifi as client
  WiFi.mode(WIFI_STA);
  //connectToWifi(); // use this for home network but not for WPA-2 Enterprise
  connectToWifiEnterprise();

  // set up mqtt 
  mqttClient.setBufferSize(512);
  mqttClient.setServer(MQTT_SERVER_PUBLIC_MOSQUITTO, MQTT_PORT_MOSQUITTO);

  connectToMQTT();

  // create tasks 
  /* xTaskCreatePinnedToCore(gpsTask, "GPSTask", 4096, NULL, 1, &gpsTaskHandle, 1);
  xTaskCreatePinnedToCore(imuTask, "IMUTask", 4096, NULL, 1, &imuTaskHandle, 1);
  // HIGH priority 
  xTaskCreatePinnedToCore(manualAlertTask, "ManualAlertTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(mqttPublishTask, "MQTTPublishTask", 8192, NULL, 2, NULL, 1); */

}

void loop() { 
  // do nothing
}