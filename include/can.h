#ifndef CAN_H
#define CAN_H


#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// CAN pins 
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// use event group to signal to publish mqtt message ONLY when one or more conditions has happened 
extern EventGroupHandle_t mqttPublishEventGroup;
#define PUBLISH_IMU_THRESHOLD_BIT (1 << 0) 
#define PUBLISH_CAN_ALERT_BIT (1 << 1) 
#define PUBLISH_MANUAL_ALERT_BIT (1 << 2) 
#define PUBLISH_MANUAL_CLEAR_BIT (1 << 3)
#define PUBLISH_HEARTBEAT_BIT (1 << 4)

bool initCAN();
bool sendCANMessage(const twai_message_t& message, int maxRetries = 3);
void handleAlertNotification(const twai_message_t& msg, QueueHandle_t alertQueue, QueueHandle_t gpsQueue, EventGroupHandle_t mqttEventGroup);
void handleAlertCleared(const twai_message_t& msg, QueueHandle_t canOutgoingQueue); 
void handleHeartbeatResponse(const twai_message_t& msg, QueueHandle_t heartbeatQueue, QueueHandle_t gpsQueue, EventGroupHandle_t mqttEventGroup);
// void aggregateAllHeartbeatData(QueueHandle_t heartbeatQueue);

#endif 