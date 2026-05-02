import socket
import threading
import time
import subprocess
import numpy as np
import cv2
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
collections.Mapping = collections.abc.Mapping
collections.Callable = collections.abc.Callable
from ultralytics import YOLO
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import sys

try:
    import msvcrt
except ImportError:
    print("This script uses msvcrt which is only available on Windows")
    sys.exit()


# ---------------- CONNECTION ----------------
connection_string = "tcp:127.0.0.1:5762"
#connection_string = "udp:127.0.0.1:14552"
print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(connection_string, wait_ready=True, timeout=60)
print("SUCCESS: Connected to drone!")


# ---------------- CONFIG ----------------
GST_BIN = r"C:\Program Files\gstreamer\1.0\msvc_x86_64\bin\gst-launch-1.0.exe"

PIPE_WIDTH, PIPE_HEIGHT = 320, 180
FRAME_SIZE = PIPE_WIDTH * PIPE_HEIGHT * 3

DISPLAY_WIDTH, DISPLAY_HEIGHT = 640, 360

PROCESS_EVERY_N_FRAMES = 3
YOLO_IMGSZ = 320

TARGET_CLASS = 0  # person
TARGET_LOST_TIMEOUT = 2.0

# Tracking / hold tuning
YAW_GAIN = 0.00175
FWD_GAIN = 0.0050
MAX_YAW_RATE = 0.35
MAX_FWD_SPEED = 1.00
MIN_FWD_SPEED = -0.35

TARGET_Y_OFFSET_RATIO = 0.245

CENTER_X_DEADBAND = 48
CENTER_Y_DEADBAND = 44
HOLD_CONFIRM_FRAMES = 8
HOLD_RELEASE_PX = 30

HOVER_REFRESH_PERIOD = 1.0
AUTO_RESUME_ZERO_BURSTS = 5

TARGET_TAKEOFF_ALT = 6.5
TARGET_TRACK_ALT = 6.5
TRACK_ALT_TOLERANCE = 1.5

TX_PRIORITY_MODES = {"LOITER", "RTL"}
AUTONOMOUS_MODES = {"AUTO", "GUIDED"}

latest_frame = None
frame_lock = threading.Lock()
state_lock = threading.Lock()

running = True
slow_landing_requested = False
immediate_abort_requested = False
manual_override_active = False

vx = 0.0
vy = 0.0
vz = 0.0
yr = 0.0


# ---------------- MODE / MOTION HELPERS ----------------
def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wait_for_mode(mode_name, timeout=10.0):
    start = time.time()
    while time.time() - start < timeout:
        if vehicle.mode.name == mode_name:
            return True
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def set_vehicle_mode(mode_name, timeout=10.0):
    global manual_override_active

    # Never allow companion to take back AUTO/GUIDED while TX override is active
    if manual_override_active and mode_name in AUTONOMOUS_MODES:
        print(f"[Mode Blocked] Manual override active. Refusing switch to {mode_name}.")
        return False

    # Only allow AUTO/GUIDED requests if vehicle is not currently in TX priority mode
    live_mode = vehicle.mode.name
    if live_mode in TX_PRIORITY_MODES and mode_name in AUTONOMOUS_MODES:
        print(f"[Mode Blocked] Vehicle is in {live_mode}. Refusing switch to {mode_name}.")
        return False

    if vehicle.mode.name != mode_name:
        print(f"[Mode] Switching to {mode_name}...")
        vehicle.mode = VehicleMode(mode_name)
        if not wait_for_mode(mode_name, timeout=timeout):
            print(f"[Mode Warning] Vehicle did not confirm {mode_name} within timeout.")

    return vehicle.mode.name == mode_name


def send_movement_command(velocity_x, velocity_y, velocity_z, yaw_rate=0.0):
    """
    Sends velocity commands to the drone using BODY_NED.
    x = forward/back
    y = right/left
    z = down/up   (+down, -up)
    yaw_rate = rad/s
    """
    if manual_override_active:
        return

    # Re-check live mode every send
    live_mode = vehicle.mode.name
    if live_mode != "GUIDED":
        return

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b011111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def set_all_zero():
    global vx, vy, vz, yr
    with state_lock:
        vx = 0.0
        vy = 0.0
        vz = 0.0
        yr = 0.0


def reset_vehicle_motion():
    set_all_zero()
    for _ in range(AUTO_RESUME_ZERO_BURSTS):
        send_movement_command(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.1)


def hold_current_position():
    loc = vehicle.location.global_relative_frame
    return LocationGlobalRelative(loc.lat, loc.lon, loc.alt)


def goto_hover_point(location):
    if manual_override_active:
        return

    live_mode = vehicle.mode.name
    if live_mode != "GUIDED":
        return

    try:
        vehicle.simple_goto(location)
    except Exception as e:
        print(f"[Hover Warning] simple_goto failed: {e}")


def return_to_auto_survey(reason="No target"):
    print(f"[Survey] {reason}. Returning to AUTO survey.")
    reset_vehicle_motion()
    set_vehicle_mode("AUTO")


def clear_autonomy_state():
    global tracking_active, hover_active, survey_mode_active
    global centered_counter, hover_point, hover_reference_center, last_target

    tracking_active = False
    hover_active = False
    survey_mode_active = False
    centered_counter = 0
    hover_point = None
    hover_reference_center = None
    last_target = None


def enter_manual_override(current_mode):
    global manual_override_active
    manual_override_active = True
    print(f"[TX Override] Pilot selected {current_mode}. Autonomous control suspended.")
    reset_vehicle_motion()


def release_manual_override():
    global manual_override_active
    manual_override_active = False
    print("[TX Override] Pilot returned to AUTO. Survey autonomy re-enabled.")
    reset_vehicle_motion()


def at_tracking_altitude():
    try:
        alt = vehicle.location.global_relative_frame.alt
        return abs(alt - TARGET_TRACK_ALT) <= TRACK_ALT_TOLERANCE
    except Exception:
        return False


def should_start_autonomy():
    """
    Optional startup gate:
    Only allow automatic takeoff/survey start if the pilot is already in AUTO.
    If the pilot is in LOITER/RTL or any other mode, do not take over.
    """
    live_mode = vehicle.mode.name
    return live_mode == "AUTO"


# ---------------- TELEMETRY LINK WATCHDOG ----------------
def telemetry_link_watchdog():
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect(("192.168.2.1", 5760))
            print("[Telemetry] TCP link reachable.")
            while True:
                time.sleep(1)
        except Exception as e:
            print(f"[Telemetry] Link failed: {e} - retrying in 5s...")
            time.sleep(5)


# ---------------- EMERGENCY ACTIONS ----------------
def slow_land():
    global running

    print("\nEmergency slow landing started...")
    descend_rate = 0.5

    try:
        # emergency action can still force GUIDED/LAND
        vehicle.mode = VehicleMode("GUIDED")
        wait_for_mode("GUIDED", timeout=5.0)

        while True:
            alt = vehicle.location.global_relative_frame.alt
            send_movement_command(0.0, 0.0, descend_rate, 0.0)
            print(f"Landing... Altitude: {alt:.2f} m")

            if alt <= 0.5:
                break

            time.sleep(0.2)

        print("Close to ground. Switching to LAND mode...")
        vehicle.mode = VehicleMode("LAND")

        while vehicle.armed:
            alt = vehicle.location.global_relative_frame.alt
            print(f"Final landing... Altitude: {alt:.2f} m")
            time.sleep(0.5)

    finally:
        reset_vehicle_motion()
        running = False
        print("Emergency landing complete.")


def immediate_abort():
    global running

    print("\nIMMEDIATE ABORT triggered...")

    try:
        reset_vehicle_motion()

        if vehicle.mode.name != "LAND":
            vehicle.mode = VehicleMode("LAND")

        time.sleep(0.5)

        for _ in range(20):
            alt = vehicle.location.global_relative_frame.alt
            print(f"Abort status... Altitude: {alt:.2f} m")
            reset_vehicle_motion()

            if alt <= 0.3:
                try:
                    vehicle.armed = False
                except Exception:
                    pass
                break

            time.sleep(0.2)

    finally:
        reset_vehicle_motion()
        running = False
        print("Immediate abort sequence finished.")


def keyboard_monitor():
    global slow_landing_requested, immediate_abort_requested, running

    print("\n" + "=" * 60)
    print("AUTONOMOUS SURVEY/TRACK MODE ACTIVE")
    print(" l = Emergency slow landing")
    print(" x = Immediate abort")
    print(" q = Close video window")
    print("=" * 60 + "\n")

    while running:
        if msvcrt.kbhit():
            key = msvcrt.getch().decode("utf-8", errors="ignore").lower()

            if key == "l":
                slow_landing_requested = True
                print("Emergency slow landing key pressed.")
                return

            if key == "x":
                immediate_abort_requested = True
                print("Immediate abort key pressed.")
                return

        time.sleep(0.05)


# ---------------- VIDEO ----------------
def frame_reader(proc):
    global latest_frame, running

    while running:
        raw = proc.stdout.read(FRAME_SIZE)

        if len(raw) != FRAME_SIZE:
            continue

        frame = np.frombuffer(raw, dtype=np.uint8).reshape((PIPE_HEIGHT, PIPE_WIDTH, 3)).copy()

        with frame_lock:
            latest_frame = frame


# ---------------- FLIGHT SETUP ----------------
def arm_and_takeoff(target_altitude):
    print("\nPreparing for automatic takeoff...")

    if vehicle.mode.name in TX_PRIORITY_MODES:
        print(f"[Takeoff Blocked] Vehicle is in {vehicle.mode.name}. Respecting transmitter priority.")
        return False

    set_vehicle_mode("GUIDED")
    if vehicle.mode.name != "GUIDED":
        print("[Takeoff Blocked] Could not enter GUIDED.")
        return False

    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.5)

    print(f"Taking off to {target_altitude:.1f} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f}m")
        if alt >= target_altitude * 0.95:
            print("Takeoff complete!")
            break
        time.sleep(1.0)

    reset_vehicle_motion()
    return True


# ---------------- MAIN ----------------
if __name__ == "__main__":
    threading.Thread(target=telemetry_link_watchdog, daemon=True).start()
    time.sleep(1)

    gst_cmd = [
        GST_BIN, "-q",
        "udpsrc", "port=5600",
        "!", "application/x-rtp,payload=96",
        "!", "rtpjitterbuffer", "latency=0",
        "!", "rtph264depay",
        "!", "h264parse",
        "!", "avdec_h264",
        "!", "videoconvert",
        "!", "videoscale",
        "!", "videorate",
        "!", f"video/x-raw,format=BGR,width={PIPE_WIDTH},height={PIPE_HEIGHT},framerate=10/1",
        "!", "queue", "leaky=downstream", "max-size-buffers=1",
        "!", "fdsink", "fd=1"
    ]

    print("[System] Starting low-latency GStreamer pipe...")
    proc = subprocess.Popen(
        gst_cmd,
        stdout=subprocess.PIPE,
        bufsize=10**6
    )

    print("[AI] Loading YOLO model...")
    model = YOLO("yolo26n.pt")
    try:
        model.to("cuda")
        print("[AI] Using CUDA")
    except Exception:
        print("[AI] CUDA not available, using CPU")

    threading.Thread(target=frame_reader, args=(proc,), daemon=True).start()

    frame_center_x = PIPE_WIDTH / 2.0
    frame_center_y = PIPE_HEIGHT / 2.0
    target_y = frame_center_y + (TARGET_Y_OFFSET_RATIO * PIPE_HEIGHT)

    # State machine
    survey_mode_active = False
    tracking_active = False
    hover_active = False
    hover_point = None
    hover_reference_center = None
    hover_last_refresh = 0.0
    last_detection_time = 0.0
    centered_counter = 0
    frame_count = 0
    last_boxes = []
    last_target = None

    try:
        # Respect pilot-selected mode at startup
        startup_mode = vehicle.mode.name
        print(f"[Startup] Initial flight mode: {startup_mode}")

        if startup_mode in TX_PRIORITY_MODES:
            print(f"[Startup] {startup_mode} is active. No autonomous takeover. Vision only.")
            manual_override_active = True
        elif startup_mode == "AUTO":
            print("[Startup] Vehicle already in AUTO. Autonomous survey is allowed.")
            survey_mode_active = True
        else:
            print(f"[Startup] Vehicle is in {startup_mode}. Waiting for pilot to select AUTO before autonomy.")

        key_thread = threading.Thread(target=keyboard_monitor, daemon=True)
        key_thread.start()

        print("[System] Running. LOITER/RTL have highest priority; boxes continue drawing in all modes.")

        while running:
            if slow_landing_requested:
                slow_land()
                break

            if immediate_abort_requested:
                immediate_abort()
                break

            # Always read live mode at top of loop
            current_mode = vehicle.mode.name

            # ---- HIGHEST PRIORITY: TRANSMITTER LOITER / RTL ----
            if current_mode in TX_PRIORITY_MODES:
                if not manual_override_active:
                    enter_manual_override(current_mode)
                    clear_autonomy_state()

            # If pilot explicitly returns to AUTO, allow autonomy again
            elif manual_override_active and current_mode == "AUTO":
                release_manual_override()
                clear_autonomy_state()
                survey_mode_active = True

            with frame_lock:
                frame = None if latest_frame is None else latest_frame.copy()

            if frame is None:
                time.sleep(0.002)
                continue

            frame_count += 1
            detected_target = None

            if frame_count % PROCESS_EVERY_N_FRAMES == 0:
                results = model(
                    frame,
                    classes=[TARGET_CLASS],
                    verbose=False,
                    imgsz=YOLO_IMGSZ
                )

                current_max_area = 0
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        area = (x2 - x1) * (y2 - y1)
                        if area > current_max_area:
                            current_max_area = area
                            detected_target = [int(x1), int(y1), int(x2), int(y2)]

                last_boxes = []
                if detected_target is not None:
                    last_boxes.append(tuple(detected_target))
                    last_target = detected_target
                    last_detection_time = time.time()
                else:
                    last_target = None

            now = time.time()
            target_visible = (now - last_detection_time) <= TARGET_LOST_TIMEOUT and last_target is not None

            # Re-read mode right before autonomy decisions
            live_mode = vehicle.mode.name

            # Never start tracking in LOITER/RTL or while override active
            if (
                target_visible
                and not manual_override_active
                and live_mode == "AUTO"
                and not tracking_active
                and not hover_active
            ):
                if at_tracking_altitude():
                    print("[Track] Person detected in AUTO at tracking altitude. Switching to GUIDED follow.")
                    if set_vehicle_mode("GUIDED") and vehicle.mode.name == "GUIDED":
                        survey_mode_active = False
                        tracking_active = True
                        centered_counter = 0
                        hover_point = None
                        hover_reference_center = None
                        reset_vehicle_motion()
                else:
                    alt = vehicle.location.global_relative_frame.alt
                    print(f"[Track] Person detected, but altitude {alt:.1f} m is outside tracking window.")

            elif not target_visible and (tracking_active or hover_active):
                tracking_active = False
                hover_active = False
                centered_counter = 0
                hover_point = None
                hover_reference_center = None

                if not manual_override_active and vehicle.mode.name == "GUIDED":
                    return_to_auto_survey("Target lost")
                    if vehicle.mode.name == "AUTO":
                        survey_mode_active = True

            status_text = "IDLE"

            if target_visible and last_target is not None:
                x1, y1, x2, y2 = last_target
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                error_x = cx - frame_center_x
                error_y = target_y - cy

                yaw_rate = clamp(error_x * YAW_GAIN, -MAX_YAW_RATE, MAX_YAW_RATE)
                forward_vel = clamp(error_y * FWD_GAIN, MIN_FWD_SPEED, MAX_FWD_SPEED)

                centered_h = abs(error_x) <= CENTER_X_DEADBAND
                centered_v = abs(error_y) <= CENTER_Y_DEADBAND

                if manual_override_active and vehicle.mode.name == "LOITER":
                    status_text = "TX PRIORITY LOITER"
                elif manual_override_active and vehicle.mode.name == "RTL":
                    status_text = "TX PRIORITY RTL"
                elif hover_active:
                    status_text = "GUIDED HOVER POINT"

                    if hover_reference_center is not None:
                        ref_cx, ref_cy = hover_reference_center
                        moved_again = (
                            abs(cx - ref_cx) > HOLD_RELEASE_PX or
                            abs(cy - ref_cy) > HOLD_RELEASE_PX
                        )
                    else:
                        moved_again = False

                    if moved_again:
                        print("[Hover] Person moved again. Releasing hover point and resuming follow.")
                        hover_active = False
                        tracking_active = True
                        hover_point = None
                        hover_reference_center = None
                        centered_counter = 0
                    else:
                        if not manual_override_active and vehicle.mode.name == "GUIDED":
                            reset_vehicle_motion()

                            if hover_point is not None and (now - hover_last_refresh) >= HOVER_REFRESH_PERIOD:
                                goto_hover_point(hover_point)
                                hover_last_refresh = now

                elif tracking_active:
                    status_text = "GUIDED FOLLOW"

                    # Hard guard: only send movement if still in GUIDED and not overridden
                    if not manual_override_active and vehicle.mode.name == "GUIDED":
                        if centered_h and centered_v and abs(forward_vel) < 0.25 and abs(yaw_rate) < 0.12:
                            centered_counter += 1
                        else:
                            centered_counter = 0

                        if centered_counter >= HOLD_CONFIRM_FRAMES:
                            hover_point = hold_current_position()
                            hover_reference_center = (cx, cy)
                            hover_last_refresh = 0.0
                            hover_active = True
                            tracking_active = False
                            reset_vehicle_motion()
                            goto_hover_point(hover_point)
                            print("[Hover] Target centered and reached. Holding above person.")
                        else:
                            send_movement_command(forward_vel, 0.0, 0.0, yaw_rate)

                elif live_mode == "AUTO":
                    status_text = "AUTO SURVEY"
                else:
                    status_text = live_mode

                # Always draw boxes if target is visible, regardless of mode
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.circle(frame, (int(frame_center_x), int(target_y)), 5, (255, 0, 0), -1)
                cv2.putText(
                    frame,
                    f"ex={error_x:.1f} ey={error_y:.1f} fwd={forward_vel:.2f} yaw={yaw_rate:.2f}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
            else:
                if manual_override_active and current_mode == "LOITER":
                    status_text = "TX PRIORITY LOITER"
                elif manual_override_active and current_mode == "RTL":
                    status_text = "TX PRIORITY RTL"
                elif tracking_active:
                    status_text = "GUIDED FOLLOW"
                elif hover_active:
                    status_text = "GUIDED HOVER POINT"
                elif vehicle.mode.name == "AUTO":
                    survey_mode_active = True
                    status_text = "AUTO SURVEY"
                else:
                    status_text = vehicle.mode.name

            display_frame = cv2.resize(
                frame,
                (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                interpolation=cv2.INTER_LINEAR
            )

            cv2.putText(
                display_frame,
                f"MODE: {vehicle.mode.name} | STATE: {status_text}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

            if manual_override_active:
                cv2.putText(
                    display_frame,
                    "TX OVERRIDE ACTIVE - NO MAVLINK FOLLOW COMMANDS",
                    (10, DISPLAY_HEIGHT - 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 165, 255),
                    2
                )

            cv2.imshow("OpenHD Person Follow / Hover", display_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("[System] Interrupted by user.")
        immediate_abort_requested = True
        immediate_abort()

    except Exception as e:
        print(f"[Main Error] {e}")
        immediate_abort_requested = True
        immediate_abort()

    finally:
        running = False

        try:
            proc.terminate()
        except Exception:
            pass

        cv2.destroyAllWindows()
        print("[System] Shutdown.")

        print("\nResetting outputs and closing connection...")
        try:
            reset_vehicle_motion()

            if vehicle.armed and vehicle.mode.name != "LAND" and not slow_landing_requested and not immediate_abort_requested:
                print("Safety fallback: switching to LAND mode before exit.")
                vehicle.mode = VehicleMode("LAND")
                time.sleep(1.0)
                reset_vehicle_motion()

        except Exception as e:
            print(f"Cleanup warning: {e}")

        finally:
            try:
                vehicle.close()
            except Exception as e:
                print(f"Close warning: {e}")

        print("Session closed successfully.")