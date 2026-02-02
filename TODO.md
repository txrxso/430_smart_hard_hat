# TO DO

## Table of Contents 
1. [CAN Functionality](#can_func)
2. [MQTT Functionality](#mqtt_func)


## 1. CAN Functionality <a name="can_func"></a>
### 1.1 **Handling Incoming Alerts** [DONE]
Items: 
- GATEWAY SENDS ACK AFTER AGGREGATING INTO ALERT PAYLOAD [DONE]
- Handling alert notification and parsing (for noise only) [DONE]
- Aggregating payload and pushing to `alertPublishQueue`[DONE]
- Signalling `mqttPublishTask`[DONE]
- **ONLY THE NOISE NODE SENDS AN ALERT FOR NOW BECAUSE 100-120 DB IS CLOSE TO INSTANT DAMAGE**. This would just be the CAN ID indicating it is an alert from the noise module and it is an `ALERT_NOTIFICATION`. The only data in the frame would be the noise level in decibels measured.

Scope: 
`void incomingCanTask()`

### 1.2 **Relaying a Manual Clear to Peripherals** [NEED TO TEST]
Scope: 
`void manualAlertTask()` 


Once manual clear condition detected: 
1. Send manual clear alert to outgoing CAN messages to notify other modules.The peripheral modules should then set a 1 minute timer where it ignores subsequent alerts.
- Need to push into `peripheralCanOutgoingQueue` 
- Put all `twai_transmit()` (ie. outgoing CAN stuff) under `incomingCanTask()` responsibility 
- so that one task now checks for queue of outgoing to peripheral + monitoring for incoming CAN messages

2. Signal MQTT publish for manual clear to notify dashboard. This 'alert' should be pushed into the `alertPublishQueue`.

## 2. MQTT Functionality <a name="mqtt_func"></a>

### 2.1 **Packaging Incoming Alerts into JSON Payloads** [DONE]
Verify that serializeAlert function works.

### 2.2 **Changing `modulesOnline`** [DONE]
0x02 = noise
0x03 = air quality 
```c++
// currently looks like this 
"modulesOnline": [0,0,0]
// change to be 
"modulesOnline": [0,0] 
//                ^first one = AIR QUALITY; second one = NOISE
"modulesOnline": [0, 2] // if only noise online
"modulesOnline": [3, 0] // if only aq online
"modulesOnline": [3, 2] // if both aq and noise online

```

Conditions for something to be `online`: 
- Received RTR Heartbeat Response from within last 5 minutes 
- Each heartbeat collection cycle is a fresh snapshot, so if a node didn't respond that cycle, we consider it `offline` 
- Need to add deterministic indexing so `[1,0]` if AIR QUALITY data included and `[0,1]` if only NOISE data included, but `[1,1]` if both responded. 

### 2.3 **MQTT PUBLISHES INCONSISTENT -> MQTT BROKER CUSTOMIZATION**
Right now: 
- HIVEMQ Public Broker on Port 1883 sometimes not that stable
- Try to set up TLS support with `PubSubClient`; though that library for sure doesn't include TLS support
- May need to write with `esp-mqtt` or look at `commitc209803` for `PubSubClient`
