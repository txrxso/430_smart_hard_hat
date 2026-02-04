# automated latency testing script (subscribe to same topic, and aggregate stats)
import paho.mqtt.client as mqtt
import json, os, time 
from datetime import datetime, timedelta
import pandas as pd 
from typing import List, Dict
import yaml

# CONFIGURATION
DURATION_HRS = 1 

# Load test config 
with open('test_configs/mqtt_broker.yaml', 'r') as f:
    config = yaml.safe_load(f)
    BROKER = config['private_broker']['host']
    PORT = config['private_broker']['port']
    HB_TOPIC = config['topics']['heartbeats']
    ALERTS_TOPIC = config['topics']['alerts']

    if 'private_broker' in config and 'username' in config['private_broker']:
        USERNAME = config['private_broker']['username']
        PASSWORD = config['private_broker']['password']

# store latency measurements
hb_latency_data = []
alert_latency_data = []

# global timing
start_time = datetime.now()
end_time = start_time + timedelta(hours=DURATION_HRS)
os.makedirs('data', exist_ok=True)

def parse_gps_timestamp(dt_str: str) -> datetime: 
    # convert to datetime
    # because payload has the format: "datetime":"2024/01/19,10:30:45" 
    try: 
        dt = datetime.strptime(dt_str, "%Y/%m/%d,%H:%M:%S")
        # convert to unix timestamp (ms)
        return int(dt.timestamp() * 1000)
    except Exception as e: 
        print(f"Error parsing timestamp: {e}")
        return None


def export_to_csv(hb_latency_data: List[Dict], alert_latency_data: List[Dict]): 
    # only export if not null
    start_str = start_time.strftime("%Y%m%d_%H%M%S")
    end_str = end_time.strftime("%Y%m%d_%H%M%S")
    hb_fp = f"heartbeat_latency_{start_str}_to_{end_str}.csv"
    alert_fp = f"alert_latency_{start_str}_to_{end_str}.csv"

    if hb_latency_data: 
        print("Exporting HB latency data.")
        hb_df = pd.DataFrame(hb_latency_data)
        hb_df.to_csv(os.path.join('data', hb_fp), index=False)

    if alert_latency_data:
        print("Exporting alert latency data.")
        a_df = pd.DataFrame(alert_latency_data)
        a_df.to_csv(os.path.join('data', alert_fp), index=False)


def on_mqtt_msg(client, data, msg: mqtt.MQTTMessage):
    arrival_t = int(time.time() * 1000) # current time in ms
    payload = json.loads(msg.payload)

    # get GPS timestamp 
    gateway_gps_ts = payload.get('datetime') 
    unix_ts = parse_gps_timestamp(gateway_gps_ts)

    # calculate latency 
    latency = arrival_t - unix_ts

    # store measurement 
    measurement = {
        'timestamp': datetime.now().isoformat(),
        'latency_ms': latency,
        'topic': msg.topic
    }

    if msg.topic == HB_TOPIC:
        hb_latency_data.append(measurement)
    elif msg.topic == ALERTS_TOPIC: 
        alert_latency_data.append(measurement)

            
def on_connect(client, data, flags, rc):
    if rc == 0: 
        print(f"Connected to MQTT Broker: {BROKER}:{PORT}")
        # subscribe to the topics
        client.subscribe(HB_TOPIC)
        client.subscribe(ALERTS_TOPIC)
    else: 
        print(f"Failed to connect; return code: {rc}")


if __name__ == "__main__":
    # set up 
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_mqtt_msg

    if 'USERNAME' in globals() and 'PASSWORD' in globals():
        client.username_pw_set(USERNAME, PASSWORD)

    client.connect(BROKER, PORT, keepalive=300)

    # start background thread
    client.loop_start() 

    print(f"Collecting data for {DURATION_HRS} hours.")
    try: 
        time.sleep(DURATION_HRS * 3600)

    except KeyboardInterrupt:
        print("Interrupted by user.")
        print("Exporting all existing data to csv.")
        export_to_csv(hb_latency_data, alert_latency_data)

    # after sleep completes
    print(f"Stopping collection.")
    client.loop_stop()
    client.disconnect()

    # export all data to .csv 
    print("Exporting to CSV")
    export_to_csv(hb_latency_data, alert_latency_data)
