######### official qualifier test - safety demo
# objective: prove autonomous flight, motion, and emergency kill switches
# constraint: must stop both vehicles within 10 seconds of kill command
# this uses the confirmed working pattern (stabilize + rc override)

import time # library for timing and logs
from pymavlink import mavutil # verified flight library
import v2v_bridge # our radio bridge talker

################################# test config
ESP32_PORT = "/dev/ttyUSB0" # drone radio box
DRONE_PORT = "/dev/ttyACM0" # drone controller wire
BAUD_RATE = 57600           # verified speed
TARGET_ALT = 1.3            # approx 4.2 feet
TEST_SPEED = 0.5            # m/s for demo

# throttle settings i tuned
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # spinning
THROTTLE_CLIMB = 1650 # lift
THROTTLE_HOVER = 1500 # hold

# official log for the refs
LOG_FILE = "qualifier_results_log.txt"

def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

def get_lidar_alt(master): # distance sensor logic
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
    if msg: return msg.current_distance / 100.0
    return 0.0

def get_uav_velocity(master): # groundspeed logic
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0)
    return msg.groundspeed if msg else 0.0

def set_throttle(master, pwm): # rc override heartbeat
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 0, 0, pwm, 0, 0, 0, 0, 0)

def change_mode(master, mode: str): # switch mode
    mapping = master.mode_mapping()
    if mode not in mapping: return
    mode_id = mapping[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    time.sleep(1)

def main(): # main safety check engine
    log_event("--- SYSTEM QUALIFIER TEST STARTING ---")
    
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Qualifier")
    try: bridge.connect()
    except: return

    log_event("Connecting to Drone Cube Orange...")
    master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
    master.wait_heartbeat()
    log_event("UAV Heartbeat OK.")

    # 1. UAV TEST PHASE
    log_event("\n[PHASE 1] UAV AUTONOMOUS launch & FLIGHT")
    change_mode(master, "STABILIZE")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
    time.sleep(2)
    
    while True: # climb loop
        alt = get_lidar_alt(master)
        if alt >= (TARGET_ALT * 0.9): break
        set_throttle(master, THROTTLE_CLIMB)
        time.sleep(0.1)
    
    set_throttle(master, THROTTLE_HOVER) # hold for demo
    log_event(f"UAV Takeoff Successful. Velocity: {get_uav_velocity(master)} m/s")
    
    # straight flight (override pulse)
    log_event("Initiating Straight Path Flight...")
    start_f = time.time()
    while (time.time() - start_f) < 3.0:
        set_throttle(master, THROTTLE_HOVER) # heartbeat
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
            0, 0, 0, TEST_SPEED, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.1)
    
    # kill switch test
    log_event("KILL SWITCH ACTIVATION: Triggering Emergency Land...")
    kill_t = time.time()
    change_mode(master, "LAND") # force safe landing instead of cutting power
    set_throttle(master, 0) # release control
    
    stop_v = False
    while (time.time() - kill_t) < 15:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log_event(f"STOP VERIFICATION: UAV HALTED AT T+{time.time()-kill_t:.2f}s")
            stop_v = True
            break
    if not stop_v: log_event("!!! FAILURE: UAV KILL SWITCH TIMEOUT !!!")

    # 2. UGV TEST PHASE
    log_event("\n[PHASE 2] UGV AUTONOMOUS MOTION & KILL")
    bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0)
    time.sleep(4)
    log_event("KILL SWITCH ACTIVATION: Triggering UGV Stop...")
    u_kill_t = time.time()
    bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=1)
    
    u_stop_v = False
    while (time.time() - u_kill_t) < 15:
        data = bridge.get_telemetry()
        if data and (data[4] == 0 or data[2] < 0.05):
            log_event(f"STOP VERIFICATION: UGV HALTED AT T+{time.time()-u_kill_t:.2f}s")
            u_stop_v = True
            break
        time.sleep(0.5)
    if not u_stop_v: log_event("!!! FAILURE: UGV KILL SWITCH TIMEOUT !!!")

    log_event("\n--- QUALIFIER TEST COMPLETE ---")
    bridge.stop()
    set_throttle(master, 0) # final safety

if __name__ == "__main__":
    main()
