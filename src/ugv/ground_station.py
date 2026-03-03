from dronekit import connect, VehicleMode
import time
import v2v_bridge
from pymavlink import mavutil 

# ------------------- SET THESE PORTS -------------------
# User manual ports for RPi
UGV_CONTROL_PORT = "/dev/ttyACM0"   # UGV Flight Controller
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"  # UGV ESP32 Bridge

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 1.0      # Drive speed
TELEM_SEND_HZ = 5    # Broadcast frequency

# ------------------- UGV Setup (DroneKit) -------------------
print("==========================================")
print("   UGV GROUND STATION - ROBUST ARMING")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=115200)
    print(f"[Ground] Connected! Ready to sync.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    exit()

def broadcast_status(bridge, seq):
    """Sends current state back to the UAV."""
    armed_val = 1 if vehicle.armed else 0
    # Map mode name to protocol index
    m = vehicle.mode.name
    mode_idx = v2v_bridge.MODE_INITIAL
    if m == "GUIDED": mode_idx = v2v_bridge.MODE_GUIDED
    elif m == "AUTO": mode_idx = v2v_bridge.MODE_AUTO
    elif m == "LAND": mode_idx = v2v_bridge.MODE_LAND
    
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    bridge.send_telemetry(seq, t_ms, 0.0, 0.0, armed_val, mode_idx)

def arm_and_move(bridge):
    """Robust Arm -> Disarm -> Arm sequence and then move."""
    print("\n[Ground] >>> INITIATING ROBUST ARM SEQUENCE")
    
    # 1. Wait for safety
    while not vehicle.is_armable:
        print("  [WAIT] Waiting for vehicle safety checks (GPS/IMU)...")
        broadcast_status(bridge, 0)
        time.sleep(1)
    
    # 2. Set Mode
    print(f"  [MODE] Switching {vehicle.mode.name} -> GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        broadcast_status(bridge, 0)
        time.sleep(0.1)
    
    # --- SEQUENCE: ARM -> DISARM -> ARM ---
    
    for attempt in ["FIRST ARM", "RESET DISARM", "FINAL ARM"]:
        state = True if "ARM" in attempt else False
        print(f"  [SYNC] {attempt}...")
        vehicle.armed = state
        
        # Wait for hardware to reflect the state
        timeout = time.time() + 5
        while vehicle.armed != state:
            if time.time() > timeout:
                print(f"    !!! Timeout during {attempt} !!!")
                break
            broadcast_status(bridge, 0)
            time.sleep(0.1)
        
        time.sleep(1.0) # Small pause between states

    if not vehicle.armed:
        print("!!! [ERROR] Final Arm failed. Aborting Move. !!!")
        return

    print("************************************")
    print("!!! UGV FULLY ARMED AND SYNCED !!!")
    print("************************************")
    
    # 3. Move forward
    print(f"[Ground] Moving Forward {DIST_M}m...")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, SPEED_MPS, 0, 0, 0, 0, 0, 0, 0)
    
    start_t = time.time()
    duration = DIST_M / SPEED_MPS
    
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)
    
    # 4. Stop and Safety Disarm
    print("[Ground] Stop. Safety Disarm.")
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    vehicle.armed = False
    broadcast_status(bridge, 0)
    print("[Ground] Mission Complete. Back to Standby.\n")

# ------------------- MAIN LOOP -------------------
def main():
    print(f"[Ground] Starting Bridge on {ESP32_BRIDGE_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="Ground-Station")
    try:
        bridge.connect()
        # --- REVERSE HELLO (Debug) ---
        print("[Ground] Sending Hello to Air...")
        bridge.send_message("hi flying rat . we connected")
    except Exception as e:
        print(f"!!! Bridge Serial Error: {e} !!!")
        return

    print("[Ground] READY: WAITING FOR AIR COMMAND...")
    
    seq = 0
    try:
        while True:
            # 1. Listen for debug string from UAV
            msg = bridge.get_message()
            if msg:
                print(f"\n>>> [AIR MESSAGE]: {msg}\n")
            
            # 2. READ AIR STATUS
            air_telem = bridge.get_telemetry()
            if air_telem:
                seq_a, t_ms_a, vx_a, vy_a, armed_a, mode_a = air_telem
                a_status = "ARMED" if armed_a == 1 else "DISARMED"
                print(f"    [RADIO] UAV STATUS: {a_status}")

            # 3. BROADCAST STATUS to UAV (Crucial for sync)
            broadcast_status(bridge, seq)
            seq += 1

            # 3. Listen for mission command
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
        print("[Ground] Shutdown.")
    finally:
        bridge.stop()
        vehicle.close()

if __name__ == "__main__":
    main()
