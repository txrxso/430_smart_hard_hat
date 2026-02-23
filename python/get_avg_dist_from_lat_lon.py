# UBC
# actual_coordinate = (49.260653, -123.250294)
# HOME TEST
# actual_coordinate = (49.169572, -123.123531) 

import math, csv, json

def haversine(coord1, coord2):
    R = 6371  # Earth radius in kilometers

    lat1, lon1 = coord1
    lat2, lon2 = coord2

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def get_coordinates_from_csv(csv_path: str): 
    """
    Parse CSV and return list of dicts with lat, lon, hdop per row.
    """
    records = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            payload = json.loads(row['payload'])
            records.append({
                'index': int(row['index']),
                'latitude': float(payload['latitude']),
                'longitude': float(payload['longitude']),
                'hdop': float(payload['hdop']),
                'satellites': int(payload['satellites']),
            })
    return records

def omit_high_hdop_values(records, threshold=5):
    """
    Filter out records where hdop >= threshold.
    """
    filtered = [r for r in records if r['hdop'] < threshold]
    removed = len(records) - len(filtered)
    print(f"HDOP threshold: {threshold} | Kept: {len(filtered)} | Removed: {removed}")
    return filtered

def compute_avg_error(records: list[dict], actual_coordinate: tuple) -> float:
    """
    Compute average Haversine error in metres against actual coordinate.
    """
    distances = [haversine(actual_coordinate, (r['latitude'], r['longitude'])) for r in records]
    avg_m = (sum(distances) / len(distances)) * 1000
    return avg_m

if __name__ == "__main__":
    # actual_coordinate = (49.169572, -123.123531) # HOME_TEST
    actual_coordinate = (49.260653, -123.250294) # UBC_TEST

    csv_path = r"C:\Users\teres\Projects\MPU6050_Test\python\data\heartbeat_latency_20260218_150505_to_20260218_170505.csv"
    records = get_coordinates_from_csv(csv_path)

    for threshold in [99, 10, 5, 4, 3, 2]:
        filtered = omit_high_hdop_values(records, threshold=threshold)
        if filtered:
            avg_error = compute_avg_error(filtered, actual_coordinate)
            print(f"  -> Avg location error: {avg_error:.2f} m\n")
        else:
            print(f"  -> No records remaining after filtering\n")
