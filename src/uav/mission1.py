######### mission 1 - autonomous launch & moving landing
# MISSION 1: OPERATION TOUCHDOWN 
# objective: takeoff from ugv, hold height, track moving ugv, and land on it
# constraint: no gps allowed . vision and optical flow only
# this uses the confirmed working pattern (stabilize + rc override)

import time # library for timing and logs
import cv2 # opencv for aruco detection
import numpy as np # math junk for vision
from pymavlink import mavutil # confirmed working communication library
import v2v_bridge # our radio translator

################################# competition config i set up
ESP32_PORT = "/dev/ttyUSB0" # radio bridge wire
DRONE_PORT = "/dev/ttyACM0" # drone controller wire (use COM4 if on windows)
BAUD_RATE = 57600           # confirmed working speed
ARUCO_ID = 0                # target marker on rover deck
TARGET_ALT = 1.3            # hover height in meters
RIDE_TIME_REQ = 30          # seconds to ride after landing
MISSION_TIMEOUT = 420       # 7 min deadline

# throttle settings i tuned
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning
THROTTLE_CLIMB = 1650 # power to lift off
THROTTLE_HOVER = 1500 # hold height power

# official logging file for the refs
LOG_FILE = "mission1_official_log.txt"

def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

#################### vision brain for the zed x camera

class Tracker: # helper to find the rover deck
    def __init__(self): # setup detector
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

    def find_ugv(self, frame): # find marker center pixel
        if frame is None: return None
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None and ARUCO_ID in ids:
            idx = list(ids.flatten()).index(ARUCO_ID)
            c = corners[idx][0]
            cx = np.mean(c[:, 0])
            cy = np.mean(c[:, 1])
            return (cx, cy)
        return None

############################ drone control helpers

def change_mode(master, mode: str): # switch flight mode
    mapping = master.mode_mapping()
    if mode not in mapping: return
    mode_id = mapping[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    time.sleep(1)

def set_throttle(master, pwm): # rc override for channel 3
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 0, 0, pwm, 0, 0, 0, 0, 0)

def get_lidar_alt(master): # floor distance via lidarlitev3
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
    if msg: return msg.current_distance / 100.0
    return 0.0

def send_velocity_pulse(master, vx, vy, vz): # move relative to drone body
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

def main(): # main mission boss engine
    log_event("--- MISSION 1 STARTING: WORKING MAVLINK PATTERN ---")
    
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Mission-1")
    try: bridge.connect()
    except: return

    log_event("Connecting to Drone Cube Orange...")
    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    log_event("Drone Heartbeat OK.")

    log_event("Waiting for UGV radio link...")
    ugv_synced = False
    while not ugv_synced: # sync loop
        set_throttle(master, 0) # keep control released while waiting
        if bridge.get_telemetry():
            log_event("UGV Found. Communications verified.")
            ugv_synced = True
        time.sleep(1.0)

    # autonomous launch (manual throttle ramp)
    log_event("UAV START TIME: Initiating Launch...")
    change_mode(master, "STABILIZE")
    
    # arming motors
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)
    
    launch_t = time.time()
    while True: # climb loop
        alt = get_lidar_alt(master)
        if alt >= (TARGET_ALT * 0.9): break
        set_throttle(master, THROTTLE_CLIMB) # pushing up
        time.sleep(0.1)
    
    set_throttle(master, THROTTLE_HOVER) # switch to hover power
    log_event(f"UAV reached {TARGET_ALT}m altitude.")

    log_event("UGV START TIME: Commanding UGV to move...")
    bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MISSION_1, estop=0)
    
    tracker = Tracker()
    # cam = cv2.VideoCapture(0) # zed x camera
    
    log_event("Tracking UGV. Alignment Sequence Running...")
    is_home = False
    
    while not is_home: # tracking loop
        # HEARTBEAT: must keep sending throttle or it crashes
        set_throttle(master, THROTTLE_HOVER) 
        
        if (time.time() - launch_t) > MISSION_TIMEOUT:
            log_event("!!! FAILURE: TIMEOUT EXCEEDED (7 MIN) !!!")
            break
            
        target = tracker.find_ugv(None) # find rover in camera
        if target: # if detected
            send_velocity_pulse(master, 0.2, 0.0, 0.1) # tracking drift
        else: # if lost
            send_velocity_pulse(master, 0.0, 0.0, 0.0) # station hold
            
        # check for touchdown (disarmed means landed)
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log_event("UAV LANDING TIME: Touchdown confirmed.")
            is_home = True
            break
            
        time.sleep(0.1) # 10hz loop heartbeat

    if is_home: # final ride duration
        log_event("Initiating 30-Second Ride Duration...")
        set_throttle(master, 0) # release control once landed
        ride_start = time.time()
        while (time.time() - ride_start) < RIDE_TIME_REQ:
            time.sleep(1.0)
        
        log_event("RIDE DURATION TIME COMPLETE: 30s achieved.")
        log_event("MISSION 1 SUCCESSFUL")
        bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=0)
    
    bridge.stop()
    set_throttle(master, 0) # triple check safety
    disarm_drone(master) # finish up

if __name__ == "__main__":
    main()