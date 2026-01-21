#ifndef CAN_H
#define CAN_H


#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "data.h"
#include "debug.h"

// CAN pins 
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// use event group to signal to publish mqtt message ONLY when one or more conditions has happened 
extern EventGroupHandle_t mqttPublishEventGroup;
#define PUBLISH_IMU_THRESHOLD_BIT (1 << 0) 
#define PUBLISH_CAN_ALERT_BIT (1 << 1) 
#define PUBLISH_MANUAL_ALERT_BIT (1 << 2) 
#define PUBLISH_MANUAL_CLEAR_BIT (1 << 3)
#define PUBLISH_HEARTBEAT_BIT (1 << 4) // thinking of adding this to its own publish category because the mqttPublishTask will be highest priority (e.g., alert, CAN alert needs to be sent)

// define a separate health event group to monitor CAN bus health, etc. (e.g., future improvement could be battery power?)
extern EventGroupHandle_t mqttPublishHealthGroup;
#define PUBLISH_CAN_HEALTH_BIT (1 << 0) // different handles 

extern int can_fail_count; 

bool initCAN();
uint32_t buildCANID(CANPriority priority, CANMessageType type, NodeID nodeid);
CANPriority getPriorityFromID(uint32_t canID);
CANMessageType getMessageTypeFromID(uint32_t canID);
NodeID getNodeIDFromID(uint32_t canID);

esp_err_t sendHeartbeatRequest();

//  TO DO: use mutex to put these into the tasks in main.cpp
bool checkCanStatus(esp_err_t status);
void resetBusState(); 

#endif 