# Data

# Alert Payload 
This is what is sent to the MQTT broker.

| Key | Example | 
|----------|------------|
| `event`  | "NOISE_OVER_THRESHOLD" |
| `datetime` | "2026/01/19,10:30:45" |
| `latitude` | 49.2827 |
| `longitude` | -123.1207 |
| `altitude` | 15 |
| `acc` | 1.22 |
| `gyro` | 2.84 |
| `noise_db` | 120 |


# Heartbeat Payload 
This is what is sent to the MQTT broker.

| Key | Example | 
|----------|------------|
| `worker_id`  | 10 |
| `modulesOnline` | [0,1] |
| `latitude` | 49.2827 |
| `longitude` | -123.1207 |
| `altitude` | 15 |
| `hdop` | 0.8  |
| `satellites` | 12 |
| `datetime` | "2026/01/19,10:30:45" |
| `resultant_acc` | 1.22 |
| `resultant_gyro` | 2.84 |
| `aqi_uba` | TBD |
| `aqi_pm100_us` | TBD |
| `aqi_pm25_us` | TBD |
| `noise_db` | 70 |
