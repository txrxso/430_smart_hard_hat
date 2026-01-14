#include "button.h"


// static used to mark variables to be permanent but not accessible from outside scope
// volatile - mark variable can be changed like in a ISR 

// state variables
static volatile unsigned long lastPressTime = 0;
static volatile unsigned long pressStartTime = 0;
static volatile bool buttonPressed = false; // get state of button press
static volatile int pressCount = 0;

// cancel timer state 
static unsigned long cancelTimerStart = 0;
static bool cancelActive = false;

// runs when button state changes (count valid presses within time window & update state vars)
void IRAM_ATTR buttonISR() {
    unsigned long currentTime = millis();
    bool currentState = digitalRead(BUTTON_PIN) == LOW; // assuming active LOW with pull-up (pressed = low to GND)

    // RELEASED -> PRESSED
    if (currentState && !buttonPressed) {
        // button pressed down
        Serial.println("Button pressed down");
        buttonPressed = true;
        pressStartTime = currentTime;
    } 

    // PRESSED -> RELEASED
    else if (!currentState && buttonPressed) {
        // button released
        Serial.println("Button released");
        buttonPressed = false;
        unsigned long pressDuration = millis()  - pressStartTime;

        // was it a valid click? 1. long enough to not be noise; 2. short enough to not be a hold
        if (pressDuration >= BUTTON_DEBOUNCE_DELAY_MS && pressDuration < BUTTON_HOLD_TIME_MS) {
            // check if it is part of the double click by checking if within click interval
            if (millis() - lastPressTime <= BUTTON_DOUBLE_PRESS_INTERVAL_MS) {
                pressCount++;
            } else {
                pressCount = 1; // reset count if outside interval
            }

            lastPressTime = currentTime; // only update last press time on valid click
        }

    }
    
}


void buttonInit() {
    pinMode(BUTTON_PIN, INPUT_PULLUP); // assuming button connects to GND when pressed
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);
}

// get current state
bool buttonState() {
    return buttonPressed;
}

// check for double press - double press means MANUAL ALERT 
bool isButtonDoublePressed() {
    unsigned long currentTime = millis(); 

    // 2+ presses and all within interval? 
    if (pressCount >= 2 && (currentTime - lastPressTime) <= BUTTON_DOUBLE_PRESS_INTERVAL_MS) {
        pressCount = 0; // reset count
        return true;
    }

    // reset single press if window expired 
    if (pressCount == 1 && (currentTime - lastPressTime) > BUTTON_DOUBLE_PRESS_INTERVAL_MS) {
        pressCount = 0;
    }

    return false; 
}

// check for hold - hold means MANUAL CLEAR - return true when detected
bool isButtonHeld() {
    unsigned long currentTime = millis();

    if (buttonPressed && (currentTime - pressStartTime) >= BUTTON_HOLD_TIME_MS) {
        // reset state to avoid multiple triggers
        buttonPressed = false; 
        pressCount = 0;

        // wait for physical release 
        while (digitalRead(BUTTON_PIN) == LOW) {
            // do nothing, just wait
        }

        return true;
    }

    return false;
}

// check if cancel timer is active
bool isCancelActive() { 
    // auto-expire if time has passed 
    if (cancelActive && millis() - cancelTimerStart > CANCEL_TIMER_DURATION) {
        cancelActive = false;
    }

    // also check if button was held down to CLEAR. this also means this should be cancelled

    return cancelActive;

}

// resets the cancel timer for manual clear
void resetCancelTimer() {
    cancelActive = true;
    cancelTimerStart = millis();
}