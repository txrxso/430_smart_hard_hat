#include "can.h"
#include "data.h"

int can_fail_count = 0;

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

esp_err_t sendHeartbeatRequest() {
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
        #if HEARTBEAT_DEBUG
        Serial.println("Heartbeat RTR sent successfully.");
        #endif
    } else {
        #if HEARTBEAT_DEBUG
        Serial.printf("Failed to send Heartbeat RTR: 0x%X\n", status);
        #endif
    }

    return status; // return so later functions can handle retry and keepign track of failures

}

bool checkCanStatus(esp_err_t status) {
  // true = signal mqttPublishHealth something wrong with CAN
  // false = keep going
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);

  if (status_info.state == TWAI_STATE_BUS_OFF) {
    #if CAN_DEBUG 
    Serial.println("TWAI state bus off.");
    #endif
    return true;

  }

  if (status == ESP_OK) {
    // reset fail count 
    can_fail_count = 0;
  }
  else if (status == ESP_FAIL || status == ESP_ERR_INVALID_STATE) { 
    can_fail_count++;
  }
  else {
    // CAN ADD LATER (not sure if it's that important)
  }

  // check against some threshold of allowed CAN fails before signalling bool 
  // to be used to signal mqttPublishHealth
  return (can_fail_count >= 20);

}


void resetBusState() {
  // initiate bus recovery
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);

  if (status_info.state == TWAI_STATE_BUS_OFF) {
    twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);

    if (twai_initiate_recovery() == ESP_OK) { 
      uint32_t alerts; 
      twai_read_alerts(&alerts, pdMS_TO_TICKS(200));

      if (alerts & TWAI_ALERT_BUS_RECOVERED) { 
        twai_start();
        can_fail_count = 0;
        Serial.println("CAN recovery successful.");
      }
      else {
        Serial.println("CAN recovery timed out.");
      }
    }
  }

}