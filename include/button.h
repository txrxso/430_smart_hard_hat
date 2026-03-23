// for all button related stuff
#ifndef BUTTON_H
#define BUTTON_H

#define BUTTON_PIN 32 

// timing constants
#define BUTTON_DEBOUNCE_DELAY_MS 50
#define BUTTON_DOUBLE_PRESS_INTERVAL_MS 1500 // max time between presses for double press to alert
#define BUTTON_HOLD_TIME_MS 2500 // time to hold for manual clear
#define CANCEL_TIMER_DURATION (60*1000) // next minute - all alerts will be ignored

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/twai.h>
#include "data.h" // for AlertPayload
#include "imu.h" // for IMUTaskManager::State
#include "vibe.h" // for VibeManager::State

// Low-level button functions (ISR and hardware)
void buttonInit(); 
bool buttonState(); // get current state
bool isButtonDoublePressed(); // check for double press
bool isButtonHeld(); // check for hold
bool isCancelActive(); // check if cancel timer is active
void resetCancelTimer(); // resets the cancel timer for manual clear
unsigned long getButtonISRCount(); // get ISR fire count for debugging

// Manual Alert Task Manager (namespace)
namespace ManualAlertTaskManager {
    
    // State container: ManualAlertTaskManager::State
    struct State {
        // Manual alert continuous streaming state
        bool manualAlertActive;
        AlertPayload originalManualAlert; // stored GPS location/timestamp from first detection
        bool pendingAlertRequest; // true if double-press occurred during cancel window
        
        // Dependencies - pointers to avoid copies
        QueueHandle_t alertPublishQueue;
        QueueHandle_t gpsQueue;
        QueueHandle_t peripheralCanOutgoingQueue;
        EventGroupHandle_t gpsEventGroup;
        EventGroupHandle_t mqttPublishEventGroup;
        IMUTaskManager::State* imuState; // pointer to IMU state for fall detection clearing
        VibeManager::State* vibeState; // pointer to vibe state for haptic feedback
    };
    
    // Public API
    void init(State& s,
              QueueHandle_t alertPublishQueue,
              QueueHandle_t gpsQueue,
              QueueHandle_t peripheralCanOutgoingQueue,
              EventGroupHandle_t gpsEventGroup,
              EventGroupHandle_t mqttPublishEventGroup,
              IMUTaskManager::State* imuState,
              VibeManager::State* vibeState);
    
    void run(State& s); // main loop (never returns)
    
    // Access functions for state
    bool isManualAlertActive(State& s);
    void clearManualAlertActive(State& s);
}

#endif