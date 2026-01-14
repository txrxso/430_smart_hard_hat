#ifndef CAN_H
#define CAN_H


#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

void handleAlertNotification(const twai_message_t& msg, QueueHandle_t alertQueue);
void handleAlertCleared(const twai_message_t& msg, QueueHandle_t canOutgoingQueue); 
void handleHeartbeatResponse(const twai_message_t& msg, QueueHandle_t heartbeatQueue);

#endif 