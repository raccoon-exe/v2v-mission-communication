from dronekit import connect, VehicleMode
import time
import v2v_bridge
from pymavlink import mavutil 

# ------------------- SET THESE PORTS -------------------
UGV_CONTROL_PORT = "/dev/ttyACM0"  # UGV Flight Controller (Cube/Pixhawk)
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 1.0      # Increased speed for "full throttle" feel
TELEM_SEND_HZ = 5    # How often to send status back to UAV

# ------------------- UGV Setup (DroneKit) -------------------
print("==========================================")
print("   UGV GROUND STATION - MISSION READY")
print("==========================================")
print(f"[Ground] Connecting to UGV Controller at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=115200)
    print(f"[Ground] Connected! Mode: {vehicle.mode.name}, Armed: {vehicle.armed}")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    exit()

def get_mode_index():
    mode = vehicle.mode.name
    if mode == "GUIDED": return v2v_bridge.MODE_GUIDED
    if mode == "AUTO":    return v2v_bridge.MODE_AUTO
    if mode == "LAND":    return v2v_bridge.MODE_LAND
    return v2v_bridge.MODE_INITIAL

def arm_and_move(bridge):
    """Arms the UGV and moves it forward 10ft."""
    print("\n[Ground] >>> ARMING SEQUENCE STARTED")
    
    # 1. Wait for armable
    while not vehicle.is_armable:
        print("  [PLN] Waiting for vehicle safety checks (GPS/IMU)...")
        time.sleep(1)
    
    # 2. Set Mode
    print(f"  [MODE] Switching {vehicle.mode.name} -> GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(0.1)
    
    # 3. Arm
    print("  [ARM] Engaging Motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("    Waiting for motors to spin up...")
        time.sleep(0.5)
    
    print("************************************")
    print("!!! UGV ARMED AND READY TO DRIVE !!!")
    print("************************************")
    
    # 4. Move
    print(f"[Ground] Moving Forward {DIST_M}m at {SPEED_MPS}m/s...")
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, SPEED_MPS, 0, 0, 0, 0, 0, 0, 0)
    
    start_t = time.time()
    duration = DIST_M / SPEED_MPS
    
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        # Broadcast status while moving
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        bridge.send_telemetry(0, t_ms, SPEED_MPS, 0.0, 1 if vehicle.armed else 0, get_mode_index())
        time.sleep(0.1)
    
    # 5. Stop
    print("[Ground] Move Complete. Stopping and Disarming.")
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    vehicle.armed = False
    print("[Ground] Mission Complete. Returning to standby.\n")

# ------------------- MAIN LOOP -------------------
def main():
    print(f"[Ground] Starting Radio Bridge on {ESP32_BRIDGE_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="Ground-Station")
    try:
        bridge.connect()
    except Exception as e:
        print(f"!!! Bridge Error: {e} !!!")
        return

    print("[Ground] WAITING FOR AIR COMMAND...")
    
    telem_seq = 0
    try:
        while True:
            # 1. READ AIR STATUS (Optional display)
            air_telem = bridge.get_telemetry()
            
            # 2. READ DEBUG MESSAGES
            msg = bridge.get_message()
            if msg:
                print(f"\n>>> [AIR MESSAGE]: {msg}\n")
            
            # 3. BROADCAST OUR STATUS (So UAV can see if we are armed)
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            bridge.send_telemetry(telem_seq, t_ms, 0.0, 0.0, 1 if vehicle.armed else 0, get_mode_index())
            telem_seq += 1

            # 3. LISTEN FOR COMMANDS
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStopFlag = cmd
                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    print("!!! [AIR COMMAND] MOVE REQUEST RECEIVED !!!")
                    arm_and_move(bridge)
                elif eStopFlag == 1:
                    print("!!! [ABORT] EMERGENCY DISARM !!!")
                    vehicle.armed = False
            
            time.sleep(1.0 / TELEM_SEND_HZ)
            
    except KeyboardInterrupt:
        print("[Ground] User Shutdown.")
    finally:
        bridge.stop()
        vehicle.close()

if __name__ == "__main__":
    main()
