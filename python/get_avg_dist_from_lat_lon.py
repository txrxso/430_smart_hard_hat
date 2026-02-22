actual_coordinate = (49.260653, -123.250294)

# UBC Macmillan Building Test
test_coordinates = coordinates = [
    (-123.250251, 49.260619),
    (-123.250243, 49.260650),
    (-123.250255, 49.260662),
    (-123.250245, 49.260633),
    (-123.250248, 49.260632),
    (-123.250253, 49.260593),
    (-123.250231, 49.260632),
    (-123.250193, 49.260641),
    (-123.250191, 49.260644),
    (-123.250209, 49.260645),
    (-123.250207, 49.260634),
    (-123.250191, 49.260609),
    (-123.250178, 49.260587),
    (-123.250173, 49.260563),
    (-123.250173, 49.260563),
]

def haversine(coord1, coord2):
    import math

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


# Convert test_coordinates from (lon, lat) to (lat, lon) to match actual_coordinate
distances = [haversine(actual_coordinate, (lat, lon)) for lon, lat in test_coordinates]

avg_distance_km = sum(distances) / len(distances)
avg_distance_m = avg_distance_km * 1000

print(f"Individual distances (m): {[round(d * 1000, 2) for d in distances]}")
print(f"Average location error: {avg_distance_m:.2f} m")