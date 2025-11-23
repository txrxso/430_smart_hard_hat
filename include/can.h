#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// function declarations
void setupCAN();
// void onReceiveCANMessage(twai_message_t &message);

#endif 
