// for all button related stuff
#ifndef BUTTON_H
#define BUTTON_H

#define BUTTON_PIN 34 

// timing constants
#define BUTTON_DEBOUNCE_DELAY_MS 50
#define BUTTON_DOUBLE_PRESS_INTERVAL_MS 1500 // max time between presses for double press to alert
#define BUTTON_HOLD_TIME_MS 2500 // time to hold for manual clear
#define CANCEL_TIMER_DURATION (2*6*1000) // next 2 minutes - all alerts will be ignored

#include <Arduino.h>
void buttonInit(); 
bool buttonState(); // get current state
bool isButtonDoublePressed(); // check for double press
bool isButtonHeld(); // check for hold
bool isCancelActive(); // check if cancel timer is active
void resetCancelTimer(); // resets the cancel timer for manual clear

#endif