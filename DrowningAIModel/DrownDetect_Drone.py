from ultralytics import YOLO
from cvlib.object_detection import draw_bbox
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import cv2
import time
import numpy as np
import sklearn
import joblib
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import albumentations
from torch.utils.data import Dataset, DataLoader
from PIL import Image
import time
import argparse


# Define command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--source', required=True, help='Video source file name')
parser.add_argument('--connect', default='tcp:127.0.0.1:5762', help='Vehicle connection target string.')
parser.add_argument('--mock-drone', action='store_true', help='Test the CV logic without a real drone connected')

# Parse command-line arguments
args = parser.parse_args()

# --- MOCK VEHICLE FOR TESTING ---
class MockVehicle:
    class Mode:
        name = "GUIDED"
    class MessageFactory:
        def set_position_target_local_ned_encode(self, *args, **kwargs): return "velocity_msg"
        def command_long_encode(self, *args, **kwargs): return "drop_msg"
    def __init__(self):
        self.mode = self.Mode()
        self.message_factory = self.MessageFactory()
    def send_mavlink(self, msg):
        pass # Silently drop mocked MAVLink messages

# --- DRONE CONNECTION ---
if args.mock_drone:
    print("MOCK MODE: Simulating Drone Connection for Testing...")
    vehicle = MockVehicle()
else:
    print(f"Connecting to vehicle on: {args.connect}")
    vehicle = connect(args.connect, wait_ready=True)
    print("Connected to drone!")

def send_movement_command(velocity_x, velocity_y, velocity_z, yaw_rate):
    """
    Sends velocity and yaw commands to the drone via MAVLink.
    velocity_x: forward/backward (m/s)
    velocity_y: left/right (m/s)
    velocity_z: up/down (m/s)
    yaw_rate: rotation speed (rad/s)
    """
    print(f">> MAVLink CMD: Forward: {velocity_x:4.2f}m/s | Lateral: {velocity_y:4.2f}m/s | YawRate: {yaw_rate:5.3f}rad/s")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b011111000111, # Fixed Bitmask to ensure yaw_rate is ACTUALLY USED
        0, 0, 0,            # Position (ignored)
        velocity_x, velocity_y, velocity_z, # Velocities
        0, 0, 0,            # Accelerations (ignored)
        0, yaw_rate         # Yaw, Yaw rate
    )
    vehicle.send_mavlink(msg)

def drop_life_vest():
    """
    Triggers a servo motor to drop the vest.
    Assumes payload servo is connected to channel 9.
    """
    print("\n" + "="*50)
    print(">>> MAVLINK TRIGGER: DEPLOYING LIFE VEST !!! <<<")
    print("="*50 + "\n")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,       # confirmation
        9,       # Servo channel (e.g., 9)
        1500,    # PWM value (usually 1000 to 2000)
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

lb = joblib.load('lb.pkl')
class CustomCNN(nn.Module):
    def __init__(self):
        super(CustomCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, 5)  # changed 3 to 1
        self.conv2 = nn.Conv2d(16, 32, 5)
        self.conv3 = nn.Conv2d(32, 64, 3)
        self.conv4 = nn.Conv2d(64, 128, 5)
        self.fc1 = nn.Linear(128, 256)
        self.fc2 = nn.Linear(256, len(lb.classes_))
        self.pool = nn.MaxPool2d(2, 2)
    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = self.pool(F.relu(self.conv3(x)))
        x = self.pool(F.relu(self.conv4(x)))
        bs, _, _, _ = x.shape
        x = F.adaptive_avg_pool2d(x, 1).reshape(bs, -1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x


print('Loading model and label binarizer...')
lb = joblib.load('lb.pkl')
num_classes = len(lb.classes_)

model = CustomCNN()
model.load_state_dict(torch.load('model.pth', map_location='cpu'))
model.eval()

# Preprocessing: MUST match the training normalization
aug = albumentations.Compose([
    albumentations.Resize(224, 224),
    albumentations.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
])

t0 = time.time() #gives time in seconds after 1970

def detectDrowning(source):
    isDrowning = False
    fram=0
    vest_dropped = False

    #input from the camera
    if source.isdigit():
        cap = cv2.VideoCapture(int(source))  # for webcam input
    else:
        cap = cv2.VideoCapture("videos/" + source)  # for video file input

    
    if (cap.isOpened() == False):
        print('Error while trying to read video')
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2
    
    # Control Tuning variables
    target_area = 40000 # The bounding box size when drone is directly above person
    gps_lock_location = None # Store GPS lock when in deadzone

    # Initialize YOLOv26 model
    yolo_model = YOLO("yolo26n.pt") #best.pt yolo11n.pt
    
    while(cap.isOpened()):
        
        status, frame = cap.read()
        
        if not status:
            break

        # apply object detection
        # bbox, label, conf = cv.detect_common_objects(frame)
        
        # 1. Run detection looking ONLY for people (class 0)
        results = yolo_model(frame, classes=[0], verbose=False)

        # 2. Reset lists so we can fill them with new detections
        bbox = []
        label = []
        conf = []

        # 3. Process results to match the format the rest of your code expects
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get the coordinates (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Add to the lists
                bbox.append([int(x1), int(y1), int(x2), int(y2)])
                label.append("person")
                conf.append(float(box.conf[0]))
        
        # if only one person is detected, use model-based detection
        if len(bbox) == 1:
            bbox0 = bbox[0]
            #centre = np.zeros(s)
            centre = [0,0]


            for i in range(0, len(bbox)):
                centre[i] =[(bbox[i][0]+bbox[i][2])/2,(bbox[i][1]+bbox[i][3])/2 ]

            centre =[(bbox0[0]+bbox0[2])/2,(bbox0[1]+bbox0[3])/2 ]
            
            start_time = time.time()
            model.eval()
            with torch.no_grad():
                pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                pil_image = aug(image=np.array(pil_image))['image']
                if fram == 500:
                    break
                fram+=1
                pil_image = np.transpose(pil_image, (2, 0, 1)).astype(np.float32)
                pil_image = torch.tensor(pil_image, dtype=torch.float).cpu()
                pil_image = pil_image.unsqueeze(0)
                outputs = model(pil_image)
                _, preds = torch.max(outputs.data, 1)

            print("Swimming status : ",lb.classes_[preds])
            if(lb.classes_[preds] =='drowning'):
                isDrowning = True
            if(lb.classes_[preds] =='normal'):
                isDrowning = False
            out = draw_bbox(frame, bbox, label, conf,isDrowning)
            
        # if more than one person is detected, use logic-based detection
        elif len(bbox) > 1:
            # calculate the centroid of each bounding box
            centres = []
            for i in range(len(bbox)):
                bbox_i = bbox[i]
                centre_i = [(bbox_i[0] + bbox_i[2])/2, (bbox_i[1] + bbox_i[3])/2]
                centres.append(centre_i)
            
            # calculate the distance between each pair of centroids
            distances = []
            for i in range(len(centres)):
                for j in range(i+1, len(centres)):
                    dist = np.sqrt((centres[i][0] - centres[j][0])**2 + (centres[i][1] - centres[j][1])**2)
                    distances.append(dist)
            
            # if the minimum distance is less than a threshold, consider it as drowning
            if len(distances) > 0 and min(distances) < 50:
                isDrowning = True
            else:
                isDrowning = False
            out = draw_bbox(frame, bbox, label, conf, isDrowning)

        else:
            out = frame
            
        # --- DRONE CONTROL LOGIC ---
        target_bbox = None
        if len(bbox) == 1 and isDrowning:
            target_bbox = bbox[0]
        elif len(bbox) > 1 and isDrowning:
            # We just grab the first bbox for now
            target_bbox = bbox[0] 
            
        # Switch to GUIDED mode if a drowning person is detected
        if target_bbox is not None and vehicle.mode.name != "GUIDED":
            print("\n*** DROWNING PERSON DETECTED! ***")
            print("Switching Pixhawk to GUIDED mode to initiate rescue...")
            
            # Using DroneKit's built-in mode changing method instead of raw pymavlink 
            # to prevent blocking the heartbeat background thread!
            vehicle.mode = VehicleMode("GUIDED")
            print("Mode change sent! Waiting for confirmation...")
            
            # Wait for Pixhawk custom mode to update
            while vehicle.mode.name != "GUIDED":
                time.sleep(0.5)
            
            print("Successfully in GUIDED mode!")

        # If in auto/guided mode, track the person
        if target_bbox is not None and vehicle.mode.name == "GUIDED":
            # Extract coordinates
            x1, y1, x2, y2 = target_bbox
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            
            # --- CALCULATING THE "DROP ZONE" ---
            # Instead of using bounding box area (which gets heavily distorted by the 20-degree tilt), 
            # we use the vertical position (cy) to know exactly how far the person is in front of the drone.
            
            # 1. Calculate Horizontal Error (Yaw Control)
            error_x = cx - frame_center_x
            yaw_rate = error_x * 0.002       # Turn left/right to center the person
            
            # 2. Calculate Vertical Error (Pitch/Forward Velocity Control)
            # With an IMX708 (120-deg DFOV, ~73-deg VFOV) pitched down 20 degrees at 12m altitude:
            # - Frame center (cy=0 offset) looks 4.36 meters ahead
            # - The point directly underneath the drone is ~25% below the center of the frame.
            target_y = frame_center_y + (0.245 * frame_height) 
            
            # Calculate distance from ideal Y position. 
            # If cy < target_y (person is higher in frame), we need to fly forward.
            error_y_vert = target_y - cy 
            
            # Proportional controller for forward velocity based on Y-axis distance
            forward_vel = error_y_vert * 0.005
            forward_vel = max(-0.5, min(1.0, forward_vel)) # Cap speed to [-0.5, +1.0] m/s
            
            # --- GPS DEADZONE HOLD LOGIC ---
            is_centered_horizontally = abs(error_x) < 50
            is_centered_vertically = abs(error_y_vert) < 50
            
            if is_centered_horizontally and is_centered_vertically:
                if gps_lock_location is None:
                    print("\n--- Target in deadzone! Locking GPS position to prevent drift ---")
                    if not args.mock_drone:
                        gps_lock_location = vehicle.location.global_relative_frame
                        vehicle.simple_goto(gps_lock_location)
                    else:
                        gps_lock_location = "MOCK_LOCATION"
            else:
                if gps_lock_location is not None:
                    print("\n--- Target moved out of deadzone! Releasing GPS hold ---")
                    gps_lock_location = None
                
                # Track target with velocity commands
                send_movement_command(forward_vel, 0, 0, yaw_rate)
            
            # 3. Check if ready to drop vest
            # Adjusting drop threshold to land ~2 feet short of the target.
            # Because the proportional controller slows the drone down to ~0.06m/s as it approaches the center,
            # the forward throw is only ~0.1m. Dropping at 22% below center places it perfectly 2.4 feet away!
            drop_threshold_y = frame_center_y + (0.22 * frame_height)
            
            is_in_drop_zone = cy > drop_threshold_y # Trigger when Cy crosses below the drop threshold
            
            if is_centered_horizontally and is_in_drop_zone and not vest_dropped:
                print(f"Vest deployed! Bounding Box Y: {cy:.1f}, Drop Threshold Y: {drop_threshold_y:.1f}")
                send_movement_command(0, 0, 0, 0) # Stop engines to avoid drifting during drop sequence
                drop_life_vest()
                vest_dropped = True
                
        # If no drowning detected or lost target, just hover
        elif target_bbox is None and vehicle.mode.name == "GUIDED":
            if gps_lock_location is None:
                send_movement_command(0, 0, 0, 0)
            
        # display output
        cv2.imshow("Real-time object detection", out)

        # press "Q" to stop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            exit()

detectDrowning(args.source)



