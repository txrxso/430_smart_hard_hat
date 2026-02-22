#ifndef DEBUG_H
#define DEBUG_H

#define IMU_DEBUG 0 
// 2 for higher level IMU printing
#define GPS_DEBUG 1
#define MQTT_DEBUG 1 
#define CAN_DEBUG 1
// set CAN_DEBUG to 2 to also print when no message received within timeout window, which can be useful for monitoring heartbeat timeouts, etc.
#define HEARTBEAT_DEBUG 1


// flags for mocks
#define MOCK_GPS 0

#endif
