from dotenv import load_dotenv
import os, requests, time, cv2, threading
from calculations import calculate_3d_distance, calculate_azimuth_elevation, get_motor_angles, TRIPLE_FLOAT_TUPLE, DOUBLE_FLOAT_TUPLE
from typing import Optional, Dict, Any
from RallyCameraController import RallyCameraController

FEET_TO_METRES = 0.3048

load_dotenv()

RECEIVER_LATITUDE = float(os.environ.get("LATITUDE", "0.0"))
RECEIVER_LONGTIUDE = float(os.environ.get("LONGITUDE", "0.0"))
RECEIVER_ELEVATION = float(os.environ.get("ELEVATION_METRES", "0.0"))
URL = os.environ.get("URL", "")

CAMERA_HEADING = 0.0
PAN_LIMIT = 90.0
TILT_MIN = -90.0
TILT_MAX = 50.0
IS_ANGLED_BACK_45 = False

CAMERA_NODE = "/dev/video2"

session = requests.Session()

# variables for sharing target info with the video thread
shared_target_info = None
shared_target_lock = threading.Lock()

def camera_feed_thread(device_node="/dev/video2"):
    cap = cv2.VideoCapture(device_node)
    
    if not cap.isOpened():
        print("[WARN] Cannot open camera for video feed.")
        return

    # force mjpg for higher fps
    mjpg_fourcc = ord('M') | (ord('J') << 8) | (ord('P') << 16) | (ord('G') << 24)
    cap.set(cv2.CAP_PROP_FOURCC, mjpg_fourcc)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Can't receive frame. Exiting ...")
            break
            
        # draw target info
        with shared_target_lock:
            target = shared_target_info
            
        if target:
            flight = target.get('flight', 'Unknown')
            dist_km = target.get('distance_m', 0) / 1000.0
            alt_m = target.get('alt_m', 0)
            
            # draw black background boxes for better readability
            cv2.putText(frame, f"FLIGHT: {flight}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
            cv2.putText(frame, f"FLIGHT: {flight}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.putText(frame, f"DIST: {dist_km:.1f}km | ALT: {alt_m:.0f}m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(frame, f"DIST: {dist_km:.1f}km | ALT: {alt_m:.0f}m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            cv2.putText(frame, "SEARCHING FOR TARGETS...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(frame, "SEARCHING FOR TARGETS...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow('Camera Feed', frame)
        
        # Press 'q' to quit the window
        if cv2.waitKey(1) == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

def parse_altitude(aircraft: dict) -> float:
    """Safely handles string "ground" or missing altitude values."""
    alt = aircraft.get("alt_geom") or aircraft.get("alt_baro")
    if alt == "ground" or alt is None:
        return 0.0
    try:
        return float(alt) * FEET_TO_METRES
    except (ValueError, TypeError):
        return 0.0

def fetch_aircraft_list(url: str) -> list[dict]:
    """Fetches raw JSON feed."""
    try:
        resp = session.get(url, timeout=1.0)
        return resp.json().get("aircraft", [])
    except Exception as e:
        print(f"[WARN] HTTP Fetch Error: {e}")
        return []

def is_aircraft_visible(az: float, el: float) -> bool:
    motor_pan, motor_tilt = get_motor_angles(az, el, CAMERA_HEADING, IS_ANGLED_BACK_45)

    # check if physical motor angles fall within hardware limits
    if not (-PAN_LIMIT <= motor_pan <= PAN_LIMIT):
        return False

    if not (TILT_MIN <= motor_tilt <= TILT_MAX):
        return False

    return True

def select_target_aircraft(aircraft_list: list[dict], current_hex: Optional[str]) -> Optional[dict]:
    """
    Selects target with target-locking:
    - If currently tracking a plane, stick with it until it leaves or loses signal.
    - Otherwise, select the closest plane.
    """
    valid_planes = []
    
    for aircraft in aircraft_list:
        if "lat" not in aircraft or "lon" not in aircraft:
            continue
            
        alt_m = parse_altitude(aircraft)
        dist_m = calculate_3d_distance(
            (RECEIVER_LATITUDE, RECEIVER_LONGTIUDE, RECEIVER_ELEVATION),
            (aircraft["lat"], aircraft["lon"], alt_m)
        )
        
        az, el = calculate_azimuth_elevation(
            (RECEIVER_LATITUDE, RECEIVER_LONGTIUDE, RECEIVER_ELEVATION),
            (aircraft["lat"], aircraft["lon"], alt_m)
        )
        
        plane_data = {
            "hex": aircraft.get("hex"),
            "flight": aircraft.get("flight", "N/A").strip(),
            "distance_m": dist_m,
            "alt_m": alt_m,
            "lat": aircraft["lat"],
            "lon": aircraft["lon"],
            "az": az,
            "el": el
        }
        
        if is_aircraft_visible(az, el):
            valid_planes.append(plane_data)

    if not valid_planes:
        return None

    if current_hex:
        for plane in valid_planes:
            if plane["hex"] == current_hex and plane["distance_m"] < 15000:
                return plane

    valid_planes.sort(key=lambda x: x["distance_m"])
    return valid_planes[0]

def point_at_aircraft(target: dict, camera: RallyCameraController) -> None:
    motor_pan, motor_tilt = get_motor_angles(target["az"], target["el"], CAMERA_HEADING, IS_ANGLED_BACK_45)
    camera.point_at_target(motor_pan, motor_tilt)

if __name__ == "__main__":
    current_target_hex = None
    
    camera = RallyCameraController(
        is_angled_back_45=IS_ANGLED_BACK_45, 
        camera_node=CAMERA_NODE
    )
    
    video_thread = threading.Thread(
        target=camera_feed_thread, 
        args=(CAMERA_NODE,), 
        daemon=True
    )
    video_thread.start()

    while True:
        aircraft_list = fetch_aircraft_list(URL)
        target = select_target_aircraft(aircraft_list, current_target_hex)

        with shared_target_lock:
            shared_target_info = target

        if target:
            current_target_hex = target["hex"]
            print(f"Tracking: {target['flight']} ({target['hex']}) | Dist: {target['distance_m']/1000:.2f}km")
            
            point_at_aircraft(
                target,
                camera
            )
        else:
            current_target_hex = None
            print("Searching for overhead targets...")

        
        time.sleep(0.3)