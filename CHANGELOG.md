## CHANGE LOG

## January 28th, 2026 
- Implemented CAN Heartbeat RTR Functionality 
- Set wifi power configuration set to `WIFI_PS_MAX_MODEM`
- Verified CAN Heartbeat RTR functionality using MQTT Explorer 

## February 1st, 2026
- From meeting with RP on 01/31/2026, because air quality node does not send alerts, then fix `AlertPayload`to only have fields for `noise_db`, `resultant_acc`, `resultant_gyro` instead of dynamic 'measurements'
- Handling incoming alerts from noise node in `void incomingCanTask()`; includes sending ACK back to peripheral, parsing CAN frame, aggregating into `AlertPayload`, pushing it into the `alertPublishQueue`, and signalling `mqttPublishEventGroup`
- Implemented functionality to relay `MANUAL_CLEAR` condition from gateway to peripherals. 
