# automated latency testing script (subscribe to same topic, and aggregate stats)
import paho.mqtt.client as mqtt
import json, os, time, csv, sys, signal
from datetime import datetime, timedelta, timezone
import pandas as pd 
from typing import List, Dict
import yaml

# CONFIGURATION
DURATION_HRS = 2


def load_config() -> Dict:
    # Load test config 
    with open(os.path.join(os.path.dirname(__file__), 'test_configs/mqtt_broker.yaml'), 'r') as f:
        config = yaml.safe_load(f)
        broker = config['private_broker']['host']
        port = config['private_broker']['port']
        hb_topic = config['topics']['heartbeats']
        alert_topic = config['topics']['alerts']

        if 'private_broker' in config and 'username' in config['private_broker']:
            username = config['private_broker']['username']
            password = config['private_broker']['password']

    return broker, port, hb_topic, alert_topic, username, password



def log_latency(host, port, hb_topic, alert_topic, username=None, password=None): 
    # set up 
    test_data_dir = os.path.join(os.path.dirname(__file__), "data")
    os.makedirs(test_data_dir, exist_ok=True)

    hb_out_file = os.path.join(test_data_dir, f"heartbeat_latency_{start_time.strftime('%Y%m%d_%H%M%S')}_to_{end_time.strftime('%Y%m%d_%H%M%S')}.csv")
    alert_out_file = os.path.join(test_data_dir, f"alert_latency_{start_time.strftime('%Y%m%d_%H%M%S')}_to_{end_time.strftime('%Y%m%d_%H%M%S')}.csv")

    with open(hb_out_file, 'w', newline='') as hb_csv, open(alert_out_file, 'w', newline='') as alert_csv:
        hb_writer = csv.DictWriter(hb_csv, fieldnames=['index', 'latency_s', 'arrival_time', 'sent_gps_time', 'payload'])
        hb_writer.writeheader()
        
        alert_writer = csv.DictWriter(alert_csv, fieldnames=['index', 'latency_s', 'arrival_time', 'sent_gps_time', 'payload'])
        alert_writer.writeheader()

         # counters for indexing
        hb_count = [0]  
        alert_count = [0]

        def handle_exit(_signum, _frame):
            print("Exiting and saving data...")
            hb_csv.close()
            alert_csv.close()
            client.disconnect()
            sys.exit(0)

        signal.signal(signal.SIGINT, handle_exit)

        client = mqtt.Client()

        if username and password:
            client.username_pw_set(username, password)
            client.tls_set()

        def on_connect(client, userdata, flags, rc):
            if rc == 0: 
                print(f"Connected to MQTT Broker: {broker}:{port}")
                # subscribe to the topics
                client.subscribe(hb_topic)
                client.subscribe(alert_topic)
            else: 
                print(f"Failed to connect; return code: {rc}")


        def on_message(client, userdata, msg):
            arrival_t = int(datetime.now(timezone.utc).timestamp()) # current time in seconds
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)

            print(f"\n=== Message Received ===")
            print(f"Topic: {msg.topic}")
            print(f"Raw Payload: {payload}") 

            # GPS timestamp
            gateway_gps_ts = data.get("datetime", "Invalid")

            if gateway_gps_ts == "Invalid":
                print("Skipping message: no GPS fix yet")
                return  # or continue, depending on your structure

            dt = datetime.strptime(gateway_gps_ts, "%Y/%m/%d,%H:%M:%S")

            if not gateway_gps_ts or gateway_gps_ts in ['2000/00/00,00:00:00', '0000/00/00,00:00:00', 'No GPS Sync']:
                print(f"Invalid GPS timestamp: {gateway_gps_ts}. Skipping latency calculation.")
                return
            
            dt = datetime.strptime(gateway_gps_ts, "%Y/%m/%d,%H:%M:%S")
            dt = dt.replace(tzinfo=timezone.utc) # ensure it's timezone-aware in UTC
            gps_ts = int(dt.timestamp()) # convert to seconds
            
            # latency
            latency = arrival_t - gps_ts

            # depending on topic, log to diff file
            row_data = {
                'latency_s': latency,
                'arrival_time': arrival_t,
                'sent_gps_time': gps_ts,
                'payload': payload
            }

            if msg.topic == hb_topic:
                row_data['index'] = hb_count[0]
                hb_writer.writerow(row_data)
                hb_csv.flush()  # ensure data is written
                hb_count[0] += 1
            elif msg.topic == alert_topic:
                row_data['index'] = alert_count[0]
                alert_writer.writerow(row_data)
                alert_csv.flush()  # ensure data is written
                alert_count[0] += 1

            # Check if test duration has elapsed
            if datetime.now() >= end_time:
                print(f"\nTest duration complete. Collected {hb_count[0]} heartbeats and {alert_count[0]} alerts.")
                handle_exit(None, None)

        client.on_connect = on_connect
        client.on_message = on_message
        print(f"Connecting to {host}:{port}...")
        client.connect(host, port, keepalive=300)
        client.loop_forever()

    return 0


if __name__ == "__main__":
    # store latency measurements
    hb_latency_data = []
    alert_latency_data = []

    # global timing
    start_time = datetime.now()
    end_time = start_time + timedelta(hours=DURATION_HRS)
    os.makedirs('data', exist_ok=True)


    # get config from yaml
    broker, port, hb_topic, alert_topic, username, password = load_config()
    log_latency(broker, port, hb_topic, alert_topic, username, password)
