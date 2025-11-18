// IMU (MPU6050)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
# include <mpu_setup.h>

// MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_config.h"  // config and credentials
#include "mqtt_config.h"

// GPS
#include <TinyGPS++.h>
#include "gps_config.h"


// define DEBUG mode to print stuff
#define DEBUG_MODE 1 

Adafruit_MPU6050 mpu;

// for maintaining connection state
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

TinyGPSPlus gps;
// add a global pointer to the GPS serial instance
HardwareSerial gpsSerial(2);



// structure to hold IMU and GPS data
struct SensorData { 
  // IMU data
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;

  // GPS data
  double latitude;
  double longitude;
  double altitude; // in meters
  // parameters below are sent as a quality check
  float hdop; // horizontal dilution of precision (lower maeans better accuracy)
  int satellites; // number of satellites in view ()
  String dateTime; // date and time as string
};

struct PeripheralData {
  // meant for sensor data received via CAN (noise, PM sensors, and other air quality sensors)
  float pm2_5;
  float pm10;
  
};


void readIMUData(SensorData &data) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  data.accelX = a.acceleration.x;
  data.accelY = a.acceleration.y;
  data.accelZ = a.acceleration.z;

  data.gyroX = g.gyro.x;
  data.gyroY = g.gyro.y;
  data.gyroZ = g.gyro.z;
}

bool readGPSData(SensorData &data) {
  unsigned long start = millis();
  bool gpsUpdated = false;

  while (millis() - start < 1000) {
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
  }

  return gpsUpdated;
}


String createCombinedPayload(const SensorData &data) {
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

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  setupMPU(mpu);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  // setup wifi as client
  WiFi.mode(WIFI_STA);
  connectToWifi(); // use this for home network but not for WPA-2 Enterprise
  //connectToWifiEnterprise();


  // set up mqtt 
  mqttClient.setBufferSize(512);
  mqttClient.setServer(MQTT_SERVER_PUBLIC_MOSQUITTO, MQTT_PORT_MOSQUITTO);

  connectToMQTT();

}

void loop() {
  // handle mqtt keep alive + maintaining connection
  mqttClient.loop();

  SensorData sensorData;
  
  // read IMU data 
  readIMUData(sensorData);

  // read and process GPS data 
  readGPSData(sensorData);

  String payload = createCombinedPayload(sensorData);

  // print debug info
  if (DEBUG_MODE == 1) {
    Serial.println(payload.length());
    Serial.println("IMU Data:");
    Serial.print("Accel X: "); Serial.print(sensorData.accelX); Serial.print(" m/s^2, ");
    Serial.print("Accel Y: "); Serial.print(sensorData.accelY); Serial.print(" m/s^2, ");
    Serial.print("Accel Z: "); Serial.print(sensorData.accelZ); Serial.println(" m/s^2");

    Serial.print("Gyro X: "); Serial.print(sensorData.gyroX); Serial.print(" rad/s, ");
    Serial.print("Gyro Y: "); Serial.print(sensorData.gyroY); Serial.print(" rad/s, ");
    Serial.print("Gyro Z: "); Serial.print(sensorData.gyroZ); Serial.println(" rad/s");
    Serial.println();

    Serial.println("GPS Data:");
    Serial.print("Latitude: "); Serial.print(sensorData.latitude, 6); Serial.print(", ");
    Serial.print("Longitude: "); Serial.print(sensorData.longitude, 6); Serial.print(", ");
    Serial.print("Altitude: "); Serial.print(sensorData.altitude); Serial.println(" m");
    Serial.print("HDOP: "); Serial.print(sensorData.hdop); Serial.print(", ");
    Serial.print("Satellites: "); Serial.println(sensorData.satellites);
  }

  unsigned long start = millis();

  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: "); 
      Serial.println(gps.location.lng(), 6);
      Serial.print("SPEED (km/h) = "); 
      Serial.println(gps.speed.kmph()); 
      Serial.print("ALT (min)= "); 
      Serial.println(gps.altitude.meters());
      Serial.print("HDOP = "); 
      Serial.println(gps.hdop.value() / 100.0); 
      Serial.print("Satellites = "); 
      Serial.println(gps.satellites.value()); 
      Serial.print("Time in UTC: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      Serial.println("");
    }
  }

  // check mqtt connection
  if (!mqttClient.connected()) {
    Serial.println("Lost connection");
    connectToMQTT();
  }

  // publish 
  bool success = mqttClient.publish(MQTT_TOPIC_TEST, payload.c_str()); 

  if (success) {
    Serial.println("Message published successfully");
  }
  else {
    Serial.println("Publish failed!");
  }

  delay(2000);

}