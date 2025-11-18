# 430_IMU_GPS_Node

Code for the sensor module (ESP32 + IMU + GPS) that does the following:
- Reads IMU Data from the MPU6050 sensor
- Determines if an impact occurred using IMU Data
- Polls geolocation data using the Neo6M GPS module
- Creates and formats payloads to send to the MQTT broker

TO DO: 
- impact detection code
- CAN code
- FreeRTOS? Potentially using both cores at the same time and different priority of tasks. 
