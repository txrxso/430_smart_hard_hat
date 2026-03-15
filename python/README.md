# DAQ for Testing Scripts

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Run the appropriate script depending on the DAQ purpose.

---

## Scripts

### `test_duty_cycle.py`
Measures ESP32 radio transmission durations and computes duty cycle statistics (% time radio is transmitting).

**Usage:**
```bash
python test_duty_cycle.py --port COM4 --duration 3600 --scenario heartbeat_only --nodes 1
python test_duty_cycle.py --port COM4 --duration 7200 --scenario alerts_30s --nodes 3
```

**Parameters:**
- `--port`: Serial port (e.g., COM4, /dev/ttyUSB0)
- `--duration`: Test duration in seconds
- `--scenario`: Test scenario name (heartbeat_only, alerts_30s, etc.)
- `--nodes`: Number of CAN nodes in network (1-3)

**Outputs:** CSV file and JSON file in `python/data/` with transmission timing data and duty cycle analysis.

---

### `test_latency.py`
Measures MQTT message latency by subscribing to topics and computing round-trip time using NTP or local clock.

**Usage:**
```bash
python test_latency.py
```

**Configuration:**
- Edit `test_configs/mqtt_broker.yaml` to set broker details
- Modify `DURATION_HRS` and `USE_NTP_TIME` in script

**Outputs:** CSV file with timestamp, latency. No analysis is provided by this script but can be easily done using pandas.

---

### `get_avg_dist_from_lat_lon.py`
Calculates GPS accuracy by computing average distance error from known true coordinates using haversine formula. Usually, this is used after running `test_latency.py` because it records the GPS coordinates so those can be used to compare.

**Usage:**
```bash
python get_avg_dist_from_lat_lon.py
```

**Configuration:**
- Set `actual_coordinate` to known true GPS location
- Modify input data source in script

**Outputs:** Average distance error in meters/kilometers. 