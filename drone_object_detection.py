import time
import math
import sys
import cv2
from ultralytics import YOLO
import os
from threading import Thread
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
from pymavlink import mavutil
import numpy as np
from pyzbar.pyzbar import decode  

CAMERA_MATRIX = np.array([[1000, 0, 160], [0, 1000, 120], [0, 0, 1]]) 
DIST_COEFFS = np.zeros((4, 1)) 
CAMERA_HEIGHT = 0.5 

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    LASER_PIN = 17
    GPIO.setup(LASER_PIN, GPIO.OUT)
    laser_on = False
except ImportError:
    GPIO = None
    print("GPIO module not found. Laser control is disabled.")

VIDEO_SOURCE = '/dev/video0'
FRAME_WIDTH = 320  
FRAME_HEIGHT = 240 
CONFIDENCE_THRESHOLD = 0.8
FRAME_OUTPUT_DIR = 'output_frames'
DRONE_CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 57600
LOW_BATTERY_THRESHOLD = 20
TARGET_CLASS = 'red_balloon' 
MAX_DISTANCE_TO_TARGET = 5 

model = YOLO('train2/weights/best.pt')

cap = cv2.VideoCapture(VIDEO_SOURCE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H', '2', '6', '4'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

os.makedirs(FRAME_OUTPUT_DIR, exist_ok=True)

frame_count = 0
debug_mode = True
follow_mode = False  
target_locked = False  
target_location = None 
qr_code_data = None


def toggle_laser(state):
    global laser_on
    if GPIO:
        GPIO.output(LASER_PIN, state)
        laser_on = state
        print(f"Laser is{' activated' if state else ' deactivated'}.")

def decode_qr_code(frame):
    qr_codes = decode(frame)
    qr_data = None

    for qr in qr_codes:
        qr_data = qr.data.decode('utf-8')
        points = qr.polygon
        if len(points) == 4:
            pts = np.array([points], dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
            cv2.putText(frame, qr_data, (points[0][0], points[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            print(f"QR Code detected: {qr_data}")
    
    return frame, qr_data

def detect_threat(results, camera_matrix, dist_coeffs, altitude, vehicle_location):
    global follow_mode, target_location

    for r in results:
        for box in r.boxes.data.tolist():
            if r.names[int(box[5])] == TARGET_CLASS and box[4] > CONFIDENCE_THRESHOLD:
                print(f"[Warning] {TARGET_CLASS} detected! Initiating follow-up.")

                center_x = int((box[0] + box[2]) / 2)
                center_y = int((box[1] + box[3]) / 2)
                print(f"Detected {TARGET_CLASS} at pixel location: ({center_x}, {center_y})")

                pixel_coords = np.array([[center_x, center_y]], dtype=np.float32)
                normalized_coords = cv2.undistortPoints(pixel_coords, camera_matrix, dist_coeffs)

                scale_factor = altitude / CAMERA_HEIGHT  
                real_world_x = normalized_coords[0][0][0] * scale_factor
                real_world_y = normalized_coords[0][0][1] * scale_factor
                print(f"Estimated real-world position relative to drone: ({real_world_x}, {real_world_y})")

                earth_radius = 6378137.0  
                delta_lat = (real_world_y / earth_radius) * (180.0 / np.pi)
                delta_lon = (real_world_x / (earth_radius * np.cos(vehicle_location.lat * np.pi / 180.0))) * (180.0 / np.pi)

                target_lat = vehicle_location.lat + delta_lat
                target_lon = vehicle_location.lon + delta_lon
                target_location = LocationGlobalRelative(target_lat, target_lon, altitude)
                print(f"Estimated GPS target location: {target_location}")

                follow_mode = True
                return True

    return False


def center_target(frame, box):
    global target_locked
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2
    box_center_x = int((box[0] + box[2]) / 2)
    box_center_y = int((box[1] + box[3]) / 2)
    
    distance_to_center = math.sqrt((box_center_x - frame_center_x)**2 + (box_center_y - frame_center_y)**2)

    if distance_to_center < 30: 
        target_locked = True
        return True
    return False


def monitor_drone(vehicle):
    while True:
        battery = vehicle.battery.level
        print(f"Battery level: {battery}%")
        if battery <= LOW_BATTERY_THRESHOLD:
            print("Low battery! Returning home...")
            vehicle.mode = VehicleMode("RTL")
            break
        time.sleep(10)


def video_stream(vehicle):
    global frame_count, debug_mode, follow_mode, target_locked, target_location, qr_code_data

    while True:
        success, frame = cap.read()
        if not success:
            print("Video source unreadable.")
            break

        frame, qr_code_data = decode_qr_code(frame)

        results = model(frame, conf=CONFIDENCE_THRESHOLD)

        if not follow_mode:
            if not detect_threat(results, CAMERA_MATRIX, DIST_COEFFS, vehicle.location.global_relative_frame.alt, vehicle.location.global_relative_frame):
                if qr_code_data:
                    print(f"QR Code data received: {qr_code_data}")
        else:
            for r in results:
                for box in r.boxes.data.tolist():
                    if r.names[int(box[5])] == TARGET_CLASS and box[4] > CONFIDENCE_THRESHOLD:
                        if center_target(frame, box):
                            toggle_laser(True)
                            print("Target centered. Laser active!")
                            time.sleep(1)
                            toggle_laser(False)
                            follow_mode = False
                            target_location = None
                            return

        annotated_frame = results[0].plot()
        cv2.imshow('Object Detection', annotated_frame)

        output_file = os.path.join(FRAME_OUTPUT_DIR, f'frame_{frame_count:04d}.jpg')
        cv2.imwrite(output_file, annotated_frame)

        frame_count += 1
        if debug_mode:
            print(f"Frame {frame_count} processed.")

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            print("Terminating the video stream...")
            break
        elif key & 0xFF == ord('d'):
            debug_mode = not debug_mode

    cap.release()
    cv2.destroyAllWindows()


def arm_and_takeoff(aTargetAltitude, vehicle):
    while not vehicle.is_armable:
        print("Waiting for arming...")
        time.sleep(1)

    print("Arming.")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Arming...")
        time.sleep(1)

    print(f"Taking off to {aTargetAltitude} meters.")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        VTOL_alt = vehicle.location.global_relative_frame.alt
        print(f"Current altitude: {VTOL_alt} meters.")
        if VTOL_alt>=aTargetAltitude*0.95:
            print("Take-off complete.")
            break
        time.sleep(1)


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)    
    vehicle.send_mavlink(msg)
    vehicle.flush()


def get_distance_metres(location1, location2):
    dlat = location1.lat - location2.lat
    dlon = location1.lon - location2.lon
    return math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5


def connect_to_drone():
    print("Connecting to drone...")
    try:
        vehicle = connect(DRONE_CONNECTION_STRING, baud=BAUD_RATE, wait_ready=True)
        print("Connection successful.")
        return vehicle
    except APIException as e:
        print(f"Connection error: {e}")
        sys.exit()
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit()

def fly_rectangle(points, vehicle):
    global follow_mode
    for point in points:
        if follow_mode:
            print("[Patrol stopped] Following target...")
            break
        print(f"Going to target: {point}")
        vehicle.simple_goto(point)
        while True:
            current_location = vehicle.location.global_relative_frame
            distance = get_distance_metres(current_location, point)
            if distance < 1.0:
                print(f"Arrived at target: {point}")
                break
            time.sleep(1)
        time.sleep(5)  


def main():
    vehicle = connect_to_drone()

    try:
        target_altitude = float(input("Enter desired altitude for takeoff (meters): "))
        arm_and_takeoff(target_altitude, vehicle)

        points = [
            LocationGlobalRelative(41.10171, 29.02304, target_altitude),
            LocationGlobalRelative(41.10175, 29.02349, target_altitude),
            LocationGlobalRelative(41.10152, 29.02352, target_altitude),
            LocationGlobalRelative(41.10149, 29.02306, target_altitude),
            LocationGlobalRelative(41.10160, 29.02370, target_altitude),
            LocationGlobalRelative(41.10165, 29.02375, target_altitude),
            LocationGlobalRelative(41.10170, 29.02380, target_altitude),
            LocationGlobalRelative(41.10155, 29.02385, target_altitude),
            LocationGlobalRelative(41.10145, 29.02310, target_altitude),
            LocationGlobalRelative(41.10140, 29.02320, target_altitude),
            LocationGlobalRelative(41.10135, 29.02330, target_altitude),
            LocationGlobalRelative(41.10130, 29.02340, target_altitude),
        ]

        Thread(target=monitor_drone, args=(vehicle,)).start()
        Thread(target=video_stream, args=(vehicle,)).start()

        try:
            fly_rectangle(points, vehicle)
        except Exception as e:
            print(f"An error occurred during patrol: {e}")


        if not follow_mode:
            print("Patrol complete. Returning home...")
            vehicle.mode = VehicleMode("RTL")
            while vehicle.mode.name != "RTL":
                time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Disconnecting...")
        vehicle.close()
        if GPIO:
            GPIO.cleanup()


if __name__ == "__main__":
    main()
