import csv, json

import csv
import json

INPUT_CSV = "./data/heartbeat_latency_20260210_232628_to_20260211_072628.csv"
OUTPUT_KML = "output.kml"
CONNECT_AS_PATH = False  # Set to False if you only want individual points


def generate_kml(input_csv, output_kml, connect_path=True):
    placemarks = []
    path_coordinates = []

    with open(input_csv, newline='', encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            index = row["index"]
            payload = json.loads(row["payload"])

            lat = payload["latitude"]
            lon = payload["longitude"]
            alt = payload["altitude"]
            dt = payload["datetime"]

            placemark = f"""
    <Placemark>
        <name>{index}</name>
        <description>{dt} | Alt {alt} m</description>
        <Point>
            <coordinates>{lon},{lat},{alt}</coordinates>
        </Point>
    </Placemark>"""

            placemarks.append(placemark)
            path_coordinates.append(f"{lon},{lat},{alt}")

    with open(output_kml, "w", encoding="utf-8") as f:
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<name>GPS Track</name>
""")

        # Write individual points
        for p in placemarks:
            f.write(p)

        # Optional path line
        if connect_path:
            f.write("""
    <Placemark>
        <name>Path</name>
        <Style>
            <LineStyle>
                <color>ff0000ff</color> <!-- Red -->
                <width>3</width>
            </LineStyle>
        </Style>
        <LineString>
            <altitudeMode>absolute</altitudeMode>
            <coordinates>
""")
            for coord in path_coordinates:
                f.write(f"                {coord}\n")

            f.write("""            </coordinates>
        </LineString>
    </Placemark>
""")

        f.write("""
</Document>
</kml>
""")

    print(f"KML file created: {output_kml}")


if __name__ == "__main__":
    generate_kml(INPUT_CSV, OUTPUT_KML, CONNECT_AS_PATH)
