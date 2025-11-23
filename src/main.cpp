// IMU (MPU6050)
# include <mpu_setup.h>

// MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_config.h"  // config and credentials
#include "mqtt_config.h"

// GPS
#include <TinyGPS++.h>
#include "gps_config.h"

// CAN
#include "can.h"

#include <data_structs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// define DEBUG mode to print stuff
#define DEBUG_MODE 1 

Adafruit_MPU6050 mpu;

// for maintaining connection state
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

TinyGPSPlus gps;
// add a global pointer to the GPS serial instance
HardwareSerial gpsSerial(2);


// shared sensor data
static CoreData sharedData;
static SemaphoreHandle_t dataMutex = NULL;

TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t pollTaskHandle = NULL;
TaskHandle_t canTaskHandle = NULL;

String createCombinedPayload(const CoreData &data) {
  String payload = "{";

  // IMU data
  payload += "\"accelX\":" + String(data.accelX, 2) + ",";
  payload += "\"accelY\":" + String(data.accelY, 2) + ",";
  payload += "\"accelZ\":" + String(data.accelZ, 2) + ",";
  payload += "\"gyroX\":" + String(data.gyroX, 2) + ",";
  payload += "\"gyroY\":" + String(data.gyroY, 2) + ",";
  payload += "\"gyroZ\":" + String(data.gyroZ, 2) + ",";

  // GPS data
  payload += "\"latitude\":" + String(data.latitude, 6) + ",";
  payload += "\"longitude\":" + String(data.longitude, 6) + ",";
  payload += "\"altitude\":" + String(data.altitude, 2) + ",";
  payload += "\"hdop\":" + String(data.hdop, 2) + ",";
  payload += "\"satellites\":" + String(data.satellites) + ",";
  payload += "\"date time\":" + String(data.dateTime);

  payload += "}";

  return payload;
} 


bool readGPSData(CoreData &data) {
  bool gpsUpdated = false;

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
    
  if (gps.location.isUpdated()) {
    gpsUpdated = true;

    // update sensor data
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.meters();
    data.hdop = gps.hdop.value() / 100.0;
    data.satellites = gps.satellites.value();

    if (gps.date.isValid() && gps.time.isValid()) {
      data.dateTime = String(gps.date.year()) + "/" + 
                      String(gps.date.month()) + "/" + 
                      String(gps.date.day()) + "," + 
                      String(gps.time.hour()) + ":" + 
                      String(gps.time.minute()) + ":" + 
                      String(gps.time.second());
    } 
    
    else {
      data.dateTime = "Invalid";
    }
  }
  return gpsUpdated;
}


void connectToMQTT() {
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

  // non-blocking connection with timeout 
  unsigned long startTime = millis();
  const unsigned long timeout = 3000; // 3 second timeout

  while (!mqttClient.connected() && (millis() - startTime < timeout)) {
    if (mqttClient.connect("ESP32Client")) { //MQTT_USER, MQTT_PSWD
      Serial.println("connected");
      break;
    }
    else {
      Serial.print("failed, state=");
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

      delay(2000);
    }

    mqttClient.loop(); // process connection responses

  }

}

// ===============================================================================
// Tasks - FreeRTOS
// ===============================================================================

void vTaskMqttTransmit(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(100); // 100 ms delay
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {

    if (!mqttClient.connected()) {
      connectToMQTT();
    } 
    else {
      mqttClient.loop(); // maintain connection
    }

    // start with empty snapshot
    CoreData snapshot = EMPTY_COREDATA; 

    // attempt to take mutex and wait for max of 10 ticks to snapshot shared data
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE) { 
      snapshot = sharedData;
      xSemaphoreGive(dataMutex);
    }

    // build and publish payload
    String payload = createCombinedPayload(snapshot);
    if (mqttClient.connected()) { 
      bool success = mqttClient.publish(MQTT_TOPIC_TEST, payload.c_str()); 

      if (success) {
        Serial.println("Message published successfully");
      }
      else {
        Serial.println("Publish failed!");
      }

    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);

  }

}


void vTaskCanRx(void *pvParameters) {
  for(;;) {
    twai_message_t msg;
    esp_err_t status = twai_receive(&msg, pdMS_TO_TICKS(100)); // 100 ms wait

    if (status == ESP_OK) {
      // Message received — print details
      Serial.println("TWAI: Message received");
      Serial.print("ID: "); Serial.println(msg.identifier);
      Serial.print("Extended format: "); Serial.println(msg.extd ? "yes" : "no");
      Serial.print("RTR: "); Serial.println(msg.rtr ? "yes" : "no");
      Serial.print("DLC: "); Serial.println(msg.data_length_code);

      if (!(msg.rtr)) {
        for (int i = 0; i < msg.data_length_code; i++) {
          Serial.print("  data["); Serial.print(i); Serial.print("] = ");
          Serial.println(msg.data[i]);
        }
      }
    }

    else {
      Serial.println("TWAI: No message received within timeout period");
      vTaskDelay(pdMS_TO_TICKS(10)); // wait before next poll
    }

}}

// polls IMU and GPS and writes to sharedData
void vTaskPoll(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  for(;;) {

    CoreData core_data;

    // read IMU data
    readIMUData(mpu, core_data);
    Serial.println("IMU data read");

    // read GPS data
    bool gpsUpdated = readGPSData(core_data);

    // publish into sharedData using mutx 
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE) { 
      sharedData = core_data;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(xDelay);

  }

}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  setupMPU(mpu);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  setupCAN();

  // setup wifi as client
  WiFi.mode(WIFI_STA);
  connectToWifi(); // use this for home network but not for WPA-2 Enterprise
  //connectToWifiEnterprise();

  // set up mqtt 
  mqttClient.setBufferSize(512);
  mqttClient.setServer(MQTT_SERVER_PUBLIC_MOSQUITTO, MQTT_PORT_MOSQUITTO);

  connectToMQTT();

  // create mutex and tasks
  dataMutex = xSemaphoreCreateMutex();

  if (dataMutex == NULL) {
    Serial.println("ERROR: failed to create data mutex");
  } else {
    // initialize snapshot
    memset(&sharedData, 0, sizeof(sharedData));
  }

  xTaskCreatePinnedToCore(vTaskMqttTransmit, "MQTT_Transmit", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTaskPoll, "Poll_Sensors", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTaskCanRx, "CAN_Rx", 2048, NULL, 1, NULL, 1);

}

void loop() {
  // Empty. Tasks are running independently.
}