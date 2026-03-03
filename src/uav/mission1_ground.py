from pymavlink import mavutil
import time
import math
from v2v_bridge import V2VBridge

# ------------------- SET THESE PORTS -------------------
# If using SITL, change to "udp:127.0.0.1:14551"
# If using Airliner/Hardware on Jetson, set to his COM/USB port
CONNECTION_STRING = "/dev/ttyACM0" 
BAUD_RATE = 115200

# UAV ESP32 Bridge Port
ESP32_PORT = "/dev/ttyACM1"

# ------------------- Connect -------------------
print(f"[Mission 1] Connecting to {CONNECTION_STRING}...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print("[Mission 1] Heartbeat found")

# Initialize V2V Bridge
bridge = V2VBridge(ESP32_PORT, name="UAV-Bridge")
bridge.connect()

# ------------------- Mavlink Helpers (mavutil style) -------------------

def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"Unknown mode '{mode}'")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"Mode: {mode}")

def arm_drone():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 
                                 0, 0, 0, 0, 0, 0)
    print("Arming props (GROUND ONLY)...")

def disarm_drone():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 
                                 0, 0, 0, 0, 0, 0)
    print("Disarming.")

# ------------------- MAIN MISSION 1: GROUND TEST -------------------
def main():
    try:
        # 1. Prep
        change_mode("GUIDED")
        time.sleep(1)
        
        # 2. Arm (Spinning props on ground)
        arm_drone()
        time.sleep(2)
        
        # 3. COORDINATED ACTION: Tell UGV to move!
        print("[Mission] Commanding UGV to MOVE 10ft while Drone is ARMED on ground")
        bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) 

        # 4. Simulation of "Moving" time (staying armed for 5 seconds)
        print("[Mission] Simulation: Drone is staying armed for the UGV's move...")
        for i in range(50): # 5 seconds
            # Update telemetry even while on ground
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            bridge.send_telemetry(i, t_ms, 0.0, 0.0, 0, 0)
            time.sleep(0.1)

        # 5. Clean up
        print("[Mission] Test Complete.")
        disarm_drone()

    except KeyboardInterrupt:
        print("Stopping Mission 1...")
        disarm_drone()
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
