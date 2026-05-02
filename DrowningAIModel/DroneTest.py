import argparse
import cv2
import time
import numpy as np
from ultralytics import YOLO
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import sys
import os
import socket
import threading

def mavlink_keepalive():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('192.168.2.1', 5760))
        print("MAVLink connected - video starting...")
        while True:
            time.sleep(1)
    except Exception as e:
        print(f"MAVLink keepalive error: {e}")

print("Starting MAVLink keepalive thread...")
thread = threading.Thread(target=mavlink_keepalive, daemon=True)
thread.start()
time.sleep(2)

parser = argparse.ArgumentParser()
parser.add_argument('--source', default='gstreamer', help='Video source file name, camera index, or "gstreamer" (default: gstreamer)')
parser.add_argument('--connect', default='tcp:127.0.0.1:5762', help='Vehicle connection target string.')
args = parser.parse_args()

connection_string = args.connect
print(f"Connecting to vehicle on: {connection_string}")

vehicle = connect(connection_string, wait_ready=True, timeout=60)
print("SUCCESS: Connected to drone!")

def send_movement_command(velocity_x, velocity_y, velocity_z, yaw_rate=0):
    """
    Sends velocity commands to the drone using local NED frame.
    x = forward/back, y = right/left, z = down/up
    yaw_rate = turning left (-)/right (+)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b011111000111, # Bitmask correctly configured to USE yaw_rate!
        0, 0, 0,            # Position (ignored based on bitmask)
        velocity_x,         # X velocity (Forward: +, Back: -)
        velocity_y,         # Y velocity (Right: +, Left: -)
        velocity_z,         # Z velocity (Down: +, Up: -)
        0, 0, 0,            # Acceleration (ignored)
        0, yaw_rate         # Yaw, Yaw rate
    )
    vehicle.send_mavlink(msg)

# --- TAKEOFF SEQUENCE ---
print("\nPreparing for automatic takeoff...")
vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode.name != "GUIDED":
    time.sleep(0.5)

print("Arming motors...")
vehicle.armed = True
while not vehicle.armed:
    time.sleep(0.5)

target_altitude = 12
print(f"Taking off to {target_altitude} meters (safe tracking zone)...")
vehicle.simple_takeoff(target_altitude)

# Wait until the drone reaches the safe height
while True:
    print(f" Altitude: {vehicle.location.global_relative_frame.alt:.1f}m")
    if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
        print("Takeoff complete!")
        break
    time.sleep(1)


# --- CV TRACKING LOOP ---
print("\n" + "="*40)
print("👁️ CV PERSON TRACKING ACTIVATED")
print("Press 'q' in the video window to quit")
print("="*40 + "\n")

source = args.source
if source.lower() == "gstreamer":
    # Add GStreamer bin folder to PATH so OpenCV can find the DLLs
    gst_path = r"C:\Program Files\gstreamer\1.0\msvc_x86_64\bin"
    if gst_path not in os.environ["PATH"]:
        os.environ["PATH"] = gst_path + os.pathsep + os.environ["PATH"]
    
    gst_pipeline = (
        "udpsrc port=5600 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" "
        "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
    )
    print("Using GStreamer pipeline:", gst_pipeline)
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
elif source.isdigit():
    cap = cv2.VideoCapture(int(source))
else:
    if source.startswith("videos/"):
        cap = cv2.VideoCapture(source)
    else:
        cap = cv2.VideoCapture("videos/" + source)

if not cap.isOpened():
    print('Error while trying to read video')
    vehicle.close()
    sys.exit()

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
frame_center_x = frame_width / 2
frame_center_y = frame_height / 2

# GPS lock location var
gps_lock_location = None

# Initialize YOLO model
yolo_model = YOLO("yolo26n.pt")

try:
    while cap.isOpened():
        status, frame = cap.read()
        if not status:
            break

        # Run detection looking ONLY for people (class 0)
        results = yolo_model(frame, classes=[0], verbose=False)
        
        target_bbox = None
        current_max_area = 0
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                area = (x2 - x1) * (y2 - y1)
                # Pick the largest person bounding box as target
                if area > current_max_area:
                    current_max_area = area
                    target_bbox = [int(x1), int(y1), int(x2), int(y2)]
        
        if target_bbox is not None and vehicle.mode.name == "GUIDED":
            x1, y1, x2, y2 = target_bbox
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            
            # --- CALCULATING TRACKING VELOCITIES ---
            error_x = cx - frame_center_x
            yaw_rate = error_x * 0.002       # Turn left/right to center the person
            
            # We assume drone is pitched down ~20 degrees at 12m height
            target_y = frame_center_y + (0.245 * frame_height) 
            error_y_vert = target_y - cy 
            
            forward_vel = error_y_vert * 0.005
            forward_vel = max(-0.5, min(1.0, forward_vel)) # Cap speed
            
            # --- GPS DEADZONE HOLD LOGIC & ALTITUDE SAFETY ---
            is_centered_horizontally = abs(error_x) < 50
            is_centered_vertically = abs(error_y_vert) < 50
            
            current_alt = vehicle.location.global_relative_frame.alt
            
            if 10.0 <= current_alt <= 14.0:
                if is_centered_horizontally and is_centered_vertically:
                    if gps_lock_location is None:
                        print("--- Target in deadzone! Locking GPS position ---")
                        gps_lock_location = vehicle.location.global_relative_frame
                        vehicle.simple_goto(gps_lock_location)
                else:
                    if gps_lock_location is not None:
                        print("--- Target moved out of deadzone! Releasing GPS hold ---")
                        gps_lock_location = None
                    
                    # Track target with velocity commands
                    send_movement_command(forward_vel, 0, 0, yaw_rate)
            else:
                # Outside safe altitude range. Do not track.
                cv2.putText(frame, f"UNSAFE ALT: {current_alt:.1f}m!", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # We'll only print it occasionally or use terminal logs. It's fine here as cv2 text is visual.
                if gps_lock_location is None:
                    send_movement_command(0, 0, 0, 0)
            
            # Draw bounding box and target point
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Fwd: {forward_vel:.2f} Yaw: {yaw_rate:.2f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        else:
            # Hover in place if no target
            if vehicle.mode.name == "GUIDED" and gps_lock_location is None:
                send_movement_command(0, 0, 0, 0)

        cv2.imshow("Real-time Drone Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

# Reset velocity to 0 before closing to ensure it doesn't drift
send_movement_command(0, 0, 0, 0)
cap.release()
cv2.destroyAllWindows()
vehicle.close()
print("\nSession closed successfully.")