from pymavlink import mavutil
import time
from v2v_bridge import V2VBridge

# ------------------- SET THESE PORTS -------------------
UGV_CONTROL_PORT = "/dev/ttyACM1"  # UGV Flight Controller (Cube/Pixhawk)
ESP32_BRIDGE_PORT = "/dev/ttyACM0" # UGV ESP32 Bridge

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 0.5      # Ground speed

# ------------------- UGV Setup (Mavlink) -------------------
print(f"[Ground] Connecting to UGV Controller at {UGV_CONTROL_PORT}...")
master = mavutil.mavlink_connection(UGV_CONTROL_PORT, baud=115200)
master.wait_heartbeat()
print("[Ground] UGV Heartbeat found")

def set_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping: return
    mode_id = mapping[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"[Ground] Mode set: {mode}")

def arm_ugv():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 
                                 0, 0, 0, 0, 0, 0)
    print("[Ground] Arming UGV Motors...")

def send_velocity(vx, vy, vz=0.0):
    # For a rover, vx is forward speed, vy is lateral (usually 0)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

# ------------------- MAIN GROUND STATION -------------------
def main():
    # 1. Start the Radio Bridge
    bridge = V2VBridge(ESP32_BRIDGE_PORT, name="Ground-Station")
    bridge.connect()

    print("[Ground] Ready and Listening for Air Command...")
    
    try:
        while True:
            # 2. Monitor Air Telemetry
            telem = bridge.get_telemetry()
            if telem:
                seq, t_ms, vx, vy, marker, estop = telem
                # print(f"[TELEM] Air Speed={vx:.2f}")

            # 3. Handle Air Commands
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStopFlag = cmd
                
                # --- COMMAND: MOVE 10 FT ---
                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    print("!!! [AIR COMMAND] MOVE 10 FT FRONT !!!")
                    set_mode("GUIDED")
                    time.sleep(1)
                    arm_ugv()
                    time.sleep(2)
                    
                    print(f"[Ground] UGV Moving Forward {DIST_M}m...")
                    start_t = time.time()
                    duration = DIST_M / SPEED_MPS
                    
                    while (time.time() - start_t) < duration:
                        send_velocity(SPEED_MPS, 0.0, 0.0)
                        time.sleep(0.1)
                    
                    send_velocity(0, 0, 0)
                    print("[Ground] Move Complete. Disarming.")
                    master.mav.command_long_send(master.target_system, master.target_component,
                                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 
                                                 0, 0, 0, 0, 0, 0)

                # --- COMMAND: EXPLICIT ARM ---
                elif cmdVal == v2v_bridge.CMD_ARM:
                    arm_ugv()

                # --- COMMAND: EXPLICIT DISARM ---
                elif cmdVal == v2v_bridge.CMD_DISARM:
                    print("[Ground] Explicit Disarm.")
                    master.mav.command_long_send(master.target_system, master.target_component,
                                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 
                                                 0, 0, 0, 0, 0, 0)

                # --- ABORT / E-STOP ---
                elif eStopFlag == 1:
                    print("!!! [ABORT] EMERGENCY STOP !!!")
                    send_velocity(0, 0, 0)
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("[Ground] Stopping...")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
