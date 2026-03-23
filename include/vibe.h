#ifndef VIBE_H
#define VIBE_H

#include <Arduino.h>

const int motorPin = 18; // GPIO pin connected to vibration motor
const int resolution = 8; // PWM resolution (8 bits = 0-255)
const int vFreq = 5000; // PWM frequency in Hz
const int pwmChannel = 0; // PWM channel (0-15 for ESP32)
const int motorSpeed = 255; // 100% power

// Vibration patterns
enum class VibePattern {
    NONE,
    FALL_IMPACT,      // Quick double buzz
    MANUAL_ALERT,     // Continuous periodic buzz
    NOISE_ALERT,      // Single short buzz
    AIR_QUALITY_ALERT // Triple short buzz
};


// Vibration state manager
namespace VibeManager {
    struct State {
        VibePattern currentPattern;
        unsigned long lastBuzzTime;
        int buzzCount;
        bool motorOn;
        unsigned long motorStartTime;
    };
    
    inline void init(State& s) {
        ledcSetup(pwmChannel, vFreq, resolution);
        ledcAttachPin(motorPin, pwmChannel);
        s.currentPattern = VibePattern::NONE;
        s.lastBuzzTime = 0;
        s.buzzCount = 0;
        s.motorOn = false;
        s.motorStartTime = 0;
    }
    
    inline void startPattern(State& s, VibePattern pattern) {
        s.currentPattern = pattern;
        s.lastBuzzTime = millis();
        s.buzzCount = 0;
        s.motorOn = false;
    }
    
    inline void stopPattern(State& s) {
        s.currentPattern = VibePattern::NONE;
        ledcWrite(pwmChannel, 0);
        s.motorOn = false;
    }
    
    // Call this periodically (e.g., in a task loop)
    inline void update(State& s) {
        unsigned long now = millis();
        
        if (s.currentPattern == VibePattern::NONE) {
            return;
        }
        
        // Handle motor on/off timing
        if (s.motorOn) {
            unsigned long onDuration = now - s.motorStartTime;
            
            switch (s.currentPattern) {
                case VibePattern::FALL_IMPACT:
                case VibePattern::NOISE_ALERT:
                case VibePattern::AIR_QUALITY_ALERT:
                    if (onDuration >= 200) { // 200ms buzz
                        ledcWrite(pwmChannel, 0);
                        s.motorOn = false;
                        s.lastBuzzTime = now;
                        s.buzzCount++;
                    }
                    break;
                    
                case VibePattern::MANUAL_ALERT:
                    if (onDuration >= 500) { // 500ms buzz
                        ledcWrite(pwmChannel, 0);
                        s.motorOn = false;
                        s.lastBuzzTime = now;
                    }
                    break;
                    
                default:
                    break;
            }
        }
        else {
            // Motor is off, check if we should start next buzz
            unsigned long timeSinceLastBuzz = now - s.lastBuzzTime;
            
            switch (s.currentPattern) {
                case VibePattern::FALL_IMPACT:
                    // Double buzz pattern: buzz, pause 200ms, buzz, done
                    if (s.buzzCount < 2 && timeSinceLastBuzz >= 200) {
                        ledcWrite(pwmChannel, motorSpeed);
                        s.motorOn = true;
                        s.motorStartTime = now;
                    } else if (s.buzzCount >= 2) {
                        s.currentPattern = VibePattern::NONE; // pattern complete
                    }
                    break;
                    
                case VibePattern::NOISE_ALERT:
                    // Single buzz, then done
                    if (s.buzzCount == 0) {
                        ledcWrite(pwmChannel, motorSpeed);
                        s.motorOn = true;
                        s.motorStartTime = now;
                    } else {
                        s.currentPattern = VibePattern::NONE; // pattern complete
                    }
                    break;
                    
                case VibePattern::AIR_QUALITY_ALERT:
                    // Triple buzz pattern: buzz, pause 200ms, buzz, pause, buzz
                    if (s.buzzCount < 3 && timeSinceLastBuzz >= 200) {
                        ledcWrite(pwmChannel, motorSpeed);
                        s.motorOn = true;
                        s.motorStartTime = now;
                    } else if (s.buzzCount >= 3) {
                        s.currentPattern = VibePattern::NONE; // pattern complete
                    }
                    break;
                    
                case VibePattern::MANUAL_ALERT:
                    // Continuous: buzz every 1.5 seconds
                    if (timeSinceLastBuzz >= 1500) {
                        ledcWrite(pwmChannel, motorSpeed);
                        s.motorOn = true;
                        s.motorStartTime = now;
                    }
                    break;
                    
                default:
                    break;
            }
        }
    }
}

#endif 