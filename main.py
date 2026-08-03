from dotenv import load_dotenv
import os, requests, time
from calculations import calculate_3d_distance, calculate_azimuth_elevation, TRIPLE_FLOAT_TUPLE

FEET_TO_METRES = 0.3048

load_dotenv()

RECEIVER_LATITUDE = float(os.environ.get("LATITUDE", "0.0"))
RECEIVER_LONGTIUDE = float(os.environ.get("LONGITUDE", "0.0"))
RECEIVER_ELEVATION = float(os.environ.get("ELEVATION_METRES", "0.0"))
URL = os.environ.get("URL", "")


def find_closest_aircraft(piaware_url: str) -> dict | None:
    resp = requests.get(piaware_url)
    data = resp.json()
    
    closest_plane = None
    min_dist_m = float("inf")
    
    for aircraft in data.get("aircraft", []):
        # skip any planes with no location data
        if "lat" not in aircraft or "lon" not in aircraft:
            continue
        
        alt_ft = aircraft.get("alt_geom") or aircraft.get("alt_baro") or 0
        alt_m = alt_ft * FEET_TO_METRES
        
        dist_m = calculate_3d_distance(
            (RECEIVER_LATITUDE, RECEIVER_LONGTIUDE, RECEIVER_ELEVATION),
            (aircraft["lat"], aircraft["lon"], alt_m)
        )
        
        if dist_m < min_dist_m:
            min_dist_m = dist_m
            
            closest_plane = {
                "hex": aircraft.get("hex"),
                "flight": aircraft.get("flight", "N/A").strip(),
                "distance_m": dist_m,
                "distance_km": dist_m / 1000.0,
                "alt_m": alt_m,
                "lat": aircraft["lat"],
                "lon": aircraft["lon"]
            }
    
    return closest_plane

def point_at_aircraft(aircraft: TRIPLE_FLOAT_TUPLE) -> None:
    # azimuth elevation stuff here
    # point at plane etc etc
    pass

if __name__ == "__main__":
    aircraft = None
    while True:
        aircraft = find_closest_aircraft(URL)
        point_at_aircraft(
            (
                aircraft["lat"],
                aircraft["lon"],
                aircraft["alt_m"]
            )
        )
        time.sleep(2)