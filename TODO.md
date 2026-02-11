# TO DO

## Table of Contents 
1. [CAN Functionality](#can_func)
2. [MQTT Functionality](#mqtt_func)
3. [GPS Functionality](#gps_func)


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

### 2.3 **MQTT PUBLISHES INCONSISTENT -> MQTT BROKER CUSTOMIZATION** [DONE]
Right now: 
- HIVEMQ Public Broker on Port 1883 sometimes not that stable
- Try to set up TLS support with `PubSubClient`; though that library for sure doesn't include TLS support
- May need to write with `esp-mqtt` or look at `commitc209803` for `PubSubClient`
- If TLS works, then need to add to `docs/`

## 3. GPS Functionality <a name="gps_func"></a>

- Backend will receive UTC DateTime

## 3.1 GPS Time Syncing (Timestamping Fallback)
- Want to improve from current design right now, where only depends on the datetime in `gpsQueue`
- Main constraint: GPS module has 1 second granularity, so if we query at T+0.2 vs. T+0.7 seconds, it returns the same datetime string even if 500 ms elapsed.

Hierarchy of timestamp priority:
1. If fresh GPS sync (datetime changed), update baseline.
2. If GPS location updated but time unchanged -> use ticks offset from last sync.
3. If GPS timeout/failure -> continue using ticks offset from last sync

Always use datetime string, but only add elapsed seconds if required.

## 3.2 Location Loss (?)
- If GPS location data invalid, we can use an estimate of the circumference for possible search and rescue (e.g., using 3 acc and 3 gyro)


## 3.3 GPS Last Known Location Saving
- Add last GPS location to non-volatile/flash (every 5 seconds) so it remembers even if powered OFF. Can at least give last known location until new data available.


## 3.4 GPS DateTime and Latency Problem
Issue:
- Right now, GPS data in queue is 'whenever gpsTask() last happend to run last', and since we only peek from the queue, we don't have an accurate timestamp for when we need it (e.g., to publish a payload)
- Getting on average 2-3 seconds (much longer than expected)

Fix: 
- Still have GPS occasionally sampled (ie. polled). We still want this in case lose power, and can log it to non-volatile memory (e.g., section 3.3)
    - Regular reads are required to maintain baseline and prevent signal loss and restarting from a cold fix
- But add another layer, so when there is a heartbeat or alert publish event, we read the GPS data (timestamp) if available, and use that. As a fallback, if we don't have GPS data during that, then use the previously saved value
    - This is the event-triggered read 



## 3.5 GPS Accuracy Problem 

Issue: poor accuracy. <br>
**Link to coordinates**: https://earth.google.com/earth/d/1DZ2xt-77K-BA04bCjhPHQHFmnlFWUwWW?usp=sharing 

Fix: 
- Could try new module like Neo-M8N
- Try open-air walk on campus, then double-check where interference could be coming from
- What are other realistic alternatives? RTK ruled out because so expensive. <5 setup steps mean we can't install external infrastructure like beacons on site for better positioning, though that could be under 'future work'. <br>
- Can Kalman filter be implemented if only have acc (x,y,z) and gyro (x,y,z) and how to account for drift otherwise?

