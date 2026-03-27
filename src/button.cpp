#include "button.h"
#include "debug.h"
#include "can.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


// static used to mark variables to be permanent but not accessible from outside scope
// volatile - mark variable can be changed like in a ISR 

// state variables (for ISR and low-level button functions)
static volatile unsigned long lastPressTime = 0;
static volatile unsigned long pressStartTime = 0;
static volatile bool buttonPressed = false; // get state of button press
static volatile int pressCount = 0;
static volatile unsigned long isrCounter = 0; // count ISR calls for debugging

// cancel timer state (shared across tasks)
static unsigned long cancelTimerStart = 0;
static bool cancelActive = false;

// runs when button state changes (count valid presses within time window & update state vars)
void IRAM_ATTR buttonISR() {
    isrCounter++; // increment each time ISR fires (for debugging)
    unsigned long currentTime = millis();
    bool currentState = digitalRead(BUTTON_PIN) == LOW; // assuming active LOW with pull-up (pressed = low to GND)

    // RELEASED -> PRESSED
    if (currentState && !buttonPressed) {
        // button pressed down
        buttonPressed = true;
        pressStartTime = currentTime;
    } 

    // PRESSED -> RELEASED
    else if (!currentState && buttonPressed) {
        // button released
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
    pinMode(BUTTON_PIN, INPUT); //INPUT_PULLUP); // assuming button connects to GND when pressed
    
    #if BUTTON_DEBUG
    Serial.println("\n=== BUTTON INIT ===");
    Serial.print("Button pin: ");
    Serial.println(BUTTON_PIN);
    Serial.print("Initial pin state: ");
    Serial.println(digitalRead(BUTTON_PIN) ? "HIGH" : "LOW");
    Serial.print("Attaching interrupt to pin ");
    Serial.print(digitalPinToInterrupt(BUTTON_PIN));
    Serial.println("...");
    #endif
    
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);
    
    #if BUTTON_DEBUG
    Serial.println("Button interrupt attached successfully");
    Serial.println("==================\n");
    #endif
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
        #if BUTTON_DEBUG
        Serial.println("[BUTTON EVENT] Double-press detected!");
        #endif
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

        #if BUTTON_DEBUG
        Serial.println("[BUTTON EVENT] Hold detected! Waiting for release...");
        #endif

        // wait for physical release (yield to prevent watchdog timeout)
        while (digitalRead(BUTTON_PIN) == LOW) {
            vTaskDelay(pdMS_TO_TICKS(10)); // yield to other tasks and reset watchdog
        }

        #if BUTTON_DEBUG
        Serial.println("[BUTTON EVENT] Button released after hold.");
        #endif

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
    pressCount = 0; // clear any pending single presses to prevent stale double-press detection
}

// get ISR counter for debugging
unsigned long getButtonISRCount() {
    return isrCounter;
}

// ====================================
// Manual Alert Task Manager
// ====================================

namespace ManualAlertTaskManager {

// Initialize state with dependencies
void init(State& s,
          QueueHandle_t alertPublishQueue,
          QueueHandle_t gpsQueue,
          QueueHandle_t peripheralCanOutgoingQueue,
          EventGroupHandle_t gpsEventGroup,
          EventGroupHandle_t mqttPublishEventGroup,
          IMUTaskManager::State* imuState,
          VibeManager::State* vibeState) {
    
    // Initialize manual alert state
    s.manualAlertActive = false;
    s.pendingAlertRequest = false;
    s.lastAlertSendTime = 0;
    memset(&s.originalManualAlert, 0, sizeof(AlertPayload));
    
    // Store dependencies
    s.alertPublishQueue = alertPublishQueue;
    s.gpsQueue = gpsQueue;
    s.peripheralCanOutgoingQueue = peripheralCanOutgoingQueue;
    s.gpsEventGroup = gpsEventGroup;
    s.mqttPublishEventGroup = mqttPublishEventGroup;
    s.imuState = imuState;
    s.vibeState = vibeState;
    
    #if BUTTON_DEBUG
    Serial.println("[MANUAL_ALERT] Task manager initialized");
    #endif
}

// Main task loop (never returns)
void run(State& s) {
    #if BUTTON_DEBUG
    static unsigned long lastDebugPrint = 0;
    static unsigned long lastISRCount = 0;
    #endif
    
    while (true) {
        #if BUTTON_DEBUG
        // Print debug info every 10 seconds (reduce serial overhead)
        if (millis() - lastDebugPrint > 10000) {
            unsigned long currentISRCount = getButtonISRCount();
            Serial.print("[BUTTON] ISR count: ");
            Serial.print(currentISRCount);
            Serial.print(" (delta: ");
            Serial.print(currentISRCount - lastISRCount);
            Serial.print("), Pin state: ");
            Serial.print(digitalRead(BUTTON_PIN) ? "HIGH" : "LOW");
            Serial.print(", Button pressed: ");
            Serial.println(buttonState() ? "YES" : "NO");
            lastDebugPrint = millis();
            lastISRCount = currentISRCount;
        }
        #endif
        
        // Check if cancel window just expired and we have a pending alert request
        if (!isCancelActive() && s.pendingAlertRequest) {
            #if BUTTON_DEBUG
            Serial.println("[MANUAL_ALERT] Processing pending alert after cancel window expired");
            #endif
            
            s.pendingAlertRequest = false; // clear pending flag
            
            // Create and queue manual alert payload with current GPS
            AlertPayload manualAlertPayload;
            memset(&manualAlertPayload, 0, sizeof(manualAlertPayload));
            manualAlertPayload.event = MANUAL_ALERT;
            manualAlertPayload.fall_detection = 2;
            attachGPSToAlert(manualAlertPayload, s.gpsEventGroup, s.gpsQueue);
            
            // Store this as the original manual alert for continuous streaming
            s.manualAlertActive = true;
            s.originalManualAlert = manualAlertPayload;
            s.lastAlertSendTime = millis(); // initialize send timestamp
            
            if (xQueueSend(s.alertPublishQueue, &manualAlertPayload, pdMS_TO_TICKS(5)) == pdTRUE) {
                xEventGroupSetBits(s.mqttPublishEventGroup, PUBLISH_MANUAL_ALERT_BIT);
                #if BUTTON_DEBUG
                Serial.println("[MANUAL_ALERT] Pending alert sent successfully");
                #endif
            }
        }
        
        // check for NEW manual alert (double press)
        if (isButtonDoublePressed()) { 
            #if BUTTON_DEBUG
            Serial.println("[MANUAL_ALERT] Double-press detected");
            #endif
            
            // only signal if cancel timer is not active AND not already in manual alert state
            if (!isCancelActive() && !s.manualAlertActive) {
                // Create and queue manual alert payload with current GPS
                AlertPayload manualAlertPayload;
                memset(&manualAlertPayload, 0, sizeof(manualAlertPayload));
                manualAlertPayload.event = MANUAL_ALERT;
                manualAlertPayload.fall_detection = 2; // use 2 to indicate SOS b/c 1 is for fall 
                attachGPSToAlert(manualAlertPayload, s.gpsEventGroup, s.gpsQueue);
                
                // Store this as the original manual alert for continuous streaming
                s.manualAlertActive = true;
                s.originalManualAlert = manualAlertPayload;
                s.lastAlertSendTime = millis(); // initialize send timestamp
                
                #if BUTTON_DEBUG
                Serial.println("[MANUAL_ALERT] State activated - continuous streaming started");
                #endif
                
                if (xQueueSend(s.alertPublishQueue, &manualAlertPayload, pdMS_TO_TICKS(5)) == pdTRUE) {
                    // signal MQTT publish for manual alert
                    xEventGroupSetBits(s.mqttPublishEventGroup, PUBLISH_MANUAL_ALERT_BIT);
                    
                    // Start vibration pattern for manual alert
                    if (s.vibeState) {
                        VibeManager::startPattern(*s.vibeState, VibePattern::MANUAL_ALERT);
                    }
                    
                    #if BUTTON_DEBUG
                    Serial.println("[MANUAL_ALERT] First alert queued for MQTT publish");
                    #endif
                } else {
                    #if BUTTON_DEBUG
                    Serial.println("[MANUAL_ALERT] Failed to queue alert!");
                    #endif
                }
            } else {
                #if BUTTON_DEBUG
                if (isCancelActive()) {
                    Serial.println("[MANUAL_ALERT] BLOCKED - cancel timer active, queuing for later");
                    s.pendingAlertRequest = true; // queue alert for when cancel expires
                } else {
                    Serial.println("[MANUAL_ALERT] Already active - ignoring duplicate double-press");
                }
                #else
                // Set pending flag if cancel is active
                if (isCancelActive()) {
                    s.pendingAlertRequest = true;
                }
                #endif
            }
        }

        // CONTINUOUS manual alert streaming (if active and not cancelled)
        if (s.manualAlertActive && !isCancelActive()) {
            unsigned long currentTime = millis();
            
            // Only resend every 1 second
            if (currentTime - s.lastAlertSendTime >= 1000) {
                // Resend alert with original GPS location and timestamp
                AlertPayload manualAlertResend;
                manualAlertResend = s.originalManualAlert; // use stored GPS data
                
                if (xQueueSend(s.alertPublishQueue, &manualAlertResend, pdMS_TO_TICKS(5)) == pdTRUE) {
                    xEventGroupSetBits(s.mqttPublishEventGroup, PUBLISH_MANUAL_ALERT_BIT);
                    s.lastAlertSendTime = currentTime; // update last send time
                    
                    #if BUTTON_DEBUG
                    Serial.println("[MANUAL_ALERT] Alert resent (1s interval)");
                    #endif
                }
            }
        }

        // check for manual clear (button held > 2 seconds)
        if (isButtonHeld()) {
            #if BUTTON_DEBUG
            Serial.println("[MANUAL_ALERT] Button hold detected - triggering manual clear");
            #endif 
            
            // Clear manual alert state if active
            if (s.manualAlertActive) {
                clearManualAlertActive(s);
                
                // Stop vibration pattern
                if (s.vibeState) {
                    VibeManager::stopPattern(*s.vibeState);
                }
                
                #if BUTTON_DEBUG
                Serial.println("[MANUAL_ALERT] Manual alert cleared via button hold");
                #endif
            }
            
            // Clear any pending alert request (user intentionally cleared)
            s.pendingAlertRequest = false;
            
            // Clear fall detection state if active
            if (s.imuState && IMUTaskManager::isFallActive(*s.imuState)) {
                IMUTaskManager::clearFallActive(*s.imuState);
                #if IMU_DEBUG
                Serial.println("[IMU] Fall cleared via button - sending fall_detection=0");
                #endif
            }
            
            // activate cancel timer (ignore alerts for next 60 seconds)
            resetCancelTimer(); 

            // send manual clear alert to outgoing CAN messages to notify other modules
            // (gateway -> peripherals)
            twai_message_t manualClearMsg;
            manualClearMsg.identifier = buildCANID(CONTROL, ALERT_CLEARED, GATEWAY_NODE);
            manualClearMsg.extd = 0;
            manualClearMsg.rtr = 0;
            manualClearMsg.data_length_code = 0; // no payload required

            // push to outgoing queue 
            if (xQueueSend(s.peripheralCanOutgoingQueue, &manualClearMsg, pdMS_TO_TICKS(5)) == pdTRUE) {
                #if CAN_DEBUG
                Serial.println("[CAN] Manual clear message queued");
                #endif
            } else {
                #if CAN_DEBUG
                Serial.println("[CAN] Failed to queue manual clear message");
                #endif
            }
            

            // push to outgoing queue for MQTT (gateway -> dashboard)
            AlertPayload manualClearAlert;
            memset(&manualClearAlert, 0, sizeof(manualClearAlert));
            manualClearAlert.event = MANUAL_CLEAR;
            manualClearAlert.fall_detection = 0; // explicitly set fall_detection to 0
            attachGPSToAlert(manualClearAlert, s.gpsEventGroup, s.gpsQueue);
            
            if (xQueueSend(s.alertPublishQueue, &manualClearAlert, pdMS_TO_TICKS(5)) == pdTRUE) {
                #if BUTTON_DEBUG
                Serial.println("[MQTT] Manual clear alert queued");
                #endif
                
                // signal MQTT publish for manual clear
                xEventGroupSetBits(s.mqttPublishEventGroup, PUBLISH_MANUAL_CLEAR_BIT);
            } else {
                #if BUTTON_DEBUG
                Serial.println("[MQTT] Manual clear alert FAILED to queue");
                #endif
            }
        }

        // Update vibration motor patterns (non-blocking)
        if (s.vibeState) {
            VibeManager::update(*s.vibeState);
        }

        // check every 20 ms (task delay)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Check if manual alert is currently active
bool isManualAlertActive(State& s) {
    return s.manualAlertActive;
}

// Clear manual alert state (called when button held for >2 seconds)
void clearManualAlertActive(State& s) {
    if (s.manualAlertActive) {
        s.manualAlertActive = false;
        s.lastAlertSendTime = 0; // reset timer
        memset(&s.originalManualAlert, 0, sizeof(s.originalManualAlert));
        
        #if BUTTON_DEBUG
        Serial.println("[MANUAL_ALERT] State cleared");
        #endif
    }
}

} // namespace ManualAlertTaskManager