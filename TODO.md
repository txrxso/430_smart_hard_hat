# TO DO

## Table of Contents 
1. [CAN Functionality](#can_func)
2. [MQTT Functainlity](#mqtt_func)


## 1. CAN Functionality <a name="can_func"></a>
### 1.1 **Handling Incoming Alerts** 
Items: 
- GATEWAY SENDS ACK AFTER AGGREGATING INTO ALERT PAYLOAD [DONE]
- Handling alert notification and parsing
- Aggregating payload and pushing to `alertPublishQueue`
- Signalling `mqttPublishTask`

Scope: 
`void incomingCanTask()`

### 1.2 **Relaying a Manual Clear to Peripherals** 
Scope: 
`void manualAlertTask()` 


Once manual clear condition detected: 
1. Send manual clear alert to outgoing CAN messages to notify other modules.The peripheral modules should then set a 1 minute timer where it ignores subsequent alerts.
2. Signal MQTT publish for manual clear to notify dashboard. This 'alert' should be pushed into the `alertPublishQueue`.

## 2. MQTT Functionality <a name="mqtt_func"></a>


### 2.1 **Packaging Incoming Alerts into JSON Payloads**


### 2.2 **Changing `modulesOnline`**
0 = offline
1 = online

```c++
// currently looks like this 
"modulesOnline": [0,0,0]
// change to be 
"modulesOnline": [0,0] 
//                ^first one = AIR QUALITY; second one = NOISE
```

Conditions for something to be `online`: 
- Received RTR Heartbeat Response from within last 5 minutes 
- Each heartbeat collection cycle is a fresh snapshot, so if a node didn't respond that cycle, we consider it `offline` 
- Need to add deterministic indexing so `[1,0]` if AIR QUALITY data included and `[0,1]` if only NOISE data included, but `[1,1]` if both responded. 