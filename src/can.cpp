#include "can.h"
#include "data.h"

#define CAN_DEBUG 1

bool initCAN() { 
  // get back to clean state before trying
  twai_stop(); 
  twai_driver_uninstall();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // install driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      Serial.println("Driver installed");
    } else {
      Serial.println("Failed to install driver");
      return false;
    }

  // start driver
  if (twai_start() == ESP_OK) {
      Serial.println("Driver started");
    } else {
      Serial.println("Failed to start driver");
      return false;
    }

  // set up filter 

  return true;

}


uint32_t buildCANID(CANPriority priority, CANMessageType type, NodeID nodeid) {
    // return 11-bit CAN identifier from the enum types
    return ((priority & 0x07) << 8) |   // Bits 10-8: Priority (3 bits)
           ((type & 0x1F) << 3) |        // Bits 7-3: Message Type (5 bits)
           (nodeid & 0x07);              // Bits 2-0: Node ID (3 bits)
}

CANPriority getPriorityFromID(uint32_t canID) {
    return static_cast<CANPriority>((canID >> 8) & 0x07);
}

CANMessageType getMessageTypeFromID(uint32_t canID) {
    return static_cast<CANMessageType>((canID >> 3) & 0x1F);
}

NodeID getNodeIDFromID(uint32_t canID) {
    return static_cast<NodeID>(canID & 0x07);
}

void sendHeartbeatRequest() {
    twai_message_t rtr_msg;

    // clear
    memset(&rtr_msg, 0, sizeof(twai_message_t));
    
    // standard RTR frame 
    rtr_msg.extd = 0; // standard frame
    rtr_msg.rtr = 1; // remote frame request
    rtr_msg.ss = 0; // not single shot (should retry on error)
    rtr_msg.self = 0; // not self reception
    rtr_msg.dlc_non_comp = 0; // data length code compliant <= 9 
    rtr_msg.identifier = buildCANID(HEARTBEAT, HEARTBEAT_REQUEST, THIS_NODE);
    rtr_msg.data_length_code = 8; // expect 8 bytes back

    esp_err_t status = twai_transmit(&rtr_msg, pdMS_TO_TICKS(100));

    if (status == ESP_OK) {
        #if CAN_DEBUG
        Serial.println("Heartbeat RTR sent successfully.");
        #endif
    } else {
        #if CAN_DEBUG
        Serial.printf("Failed to send Heartbeat RTR: 0x%X\n", status);
        #endif
    }

}


// void handleAlertNotification(const twai_message_t& msg, QueueHandle_t alertQueue, QueueHandle_t gpsQueue) {
//     // send ACK to peripheral module 
//     Serial.println("Processing ALERT_NOTIFICATION from peripheral.");
//     twai_message_t ack_msg;
//     ack_msg.identifier = ALERT_ACK;
//     ack_msg.data_length_code = 1; 
//     ack_msg.extd = 0; // standard frame
//     ack_msg.rtr = 0; // not remote frame request 
//     ack_msg.data[0] = THIS_NODE; 

//     if (sendCANMessage(ack_msg)) {
//       Serial.println("ALERT_ACK sent successfully.");
//     } else {
//       Serial.println("Failed to send ALERT_ACK.");
//     }

//     // process alert data from msg into AlertPayload 
//     AlertPayload payload = {}; 

//     // get latest GPS coordinates and datetime from gpsQueue
//     gpsData latestGpsData;
//     if (gpsQueue != NULL && xQueuePeek(gpsQueue, &latestGpsData, 10) == pdTRUE) {
//       payload.latitude = latestGpsData.latitude;
//       payload.longitude = latestGpsData.longitude;
//       payload.altitude = latestGpsData.altitude;
//       strncpy(payload.dateTime, latestGpsData.dateTime, sizeof(payload.dateTime));
//     } else {
//       payload.latitude = 0.0;
//       payload.longitude = 0.0;
//       payload.altitude = 0.0;
//       strncpy(payload.dateTime, "Invalid", sizeof(payload.dateTime));
//     }

//     // parse measurements from CAN data


//     // push to alertPublishQueue for mqtt publish task
//     if (alertQueue != NULL) {
//       if (xQueueSend(alertQueue, &payload, pdMS_TO_TICKS(80)) == pdTRUE) {
//         Serial.println("AlertPayload pushed to alertQueue.");
//       } else {
//         Serial.println("Failed to push AlertPayload to alertQueue.");
//       }
//     }

    

// }


// void handleAlertCleared(const twai_message_t& msg, QueueHandle_t canOutgoingQueue) {

// }

// void handleHeartbeatResponse(const twai_message_t& msg, QueueHandle_t heartbeatQueue) {

// }

// void handleDefault() {
//     Serial.println("Received unknown CAN message ID.");
// }
