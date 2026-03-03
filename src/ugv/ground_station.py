from dronekit import connect, VehicleMode
import time
import math
import v2v_bridge
from pymavlink import mavutil 

# ------------------- SET THESE PORTS -------------------
UGV_CONTROL_PORT = "/dev/ttyACM0"   
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"  

# ------------------- Mission params -------------------
DIST_M = 3.048       # Default (10 ft)
SPEED_MPS = 1.0      
TELEM_SEND_HZ = 5    

# ------------------- UGV Setup -------------------
print("==========================================")
print("   UGV GROUND STATION - ZIG-ZAG READY")
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
    m = vehicle.mode.name
    mode_idx = v2v_bridge.MODE_INITIAL
    if m == "GUIDED": mode_idx = v2v_bridge.MODE_GUIDED
    elif m == "AUTO": mode_idx = v2v_bridge.MODE_AUTO
    elif m == "LAND": mode_idx = v2v_bridge.MODE_LAND
    
    armable_bit = 0x10 if vehicle.is_armable else 0x00
    gps_bit = 0x20 if (vehicle.gps_0.fix_type > 0) else 0x00
    safety_byte = (mode_idx & 0x0F) | armable_bit | gps_bit
    
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    # Speed Fix: Use groundspeed
    v_mps = vehicle.groundspeed if vehicle.groundspeed is not None else 0.0
    bridge.send_telemetry(seq, t_ms, v_mps, 0.0, armed_val, safety_byte)

def arm_and_move(bridge):
    """Bypasses safety to arm in current mode before switching to GUIDED."""
    print("\n[Ground] >>> INITIATING HYBRID ARM SEQUENCE")
    if not vehicle.is_armable:
        print(f"!!! [WARNING] PRE-ARM CHECKS FAILED (GPS: {vehicle.gps_0.fix_type}) !!!")
        print("    [NOTICE] Bypassing safety check for manual test...")
    
    for attempt in ["FIRST ARM", "RESET DISARM", "FINAL ARM"]:
        state = True if "ARM" in attempt else False
        print(f"  [FORCE-SYNC] Initiating {attempt} in {vehicle.mode.name} mode...")
        for retry in range(3):
            print(f"    - Attempt {retry+1}/3: Setting vehicle.armed to {state}")
            vehicle.armed = state
            timeout = time.time() + 3
            while vehicle.armed != state:
                if time.time() > timeout: break
                broadcast_status(bridge, 0)
                time.sleep(0.1)
            if vehicle.armed == state:
                print(f"    - Success: Vehicle is now {'Armed' if state else 'Disarmed'}")
                break
        time.sleep(1.0) 

    if not vehicle.armed:
        print("!!! [CRITICAL] UGV failed to ARM. !!!")
        return

    print(f"  [MODE] Switching {vehicle.mode.name} -> GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    m_timeout = time.time() + 5
    while vehicle.mode.name != "GUIDED" and time.time() < m_timeout:
        broadcast_status(bridge, 0)
        time.sleep(0.1)
    
    print("************************************")
    print("!!! UGV FULLY ARMED AND SYNCED !!!")
    print("************************************")
    print("[Ground] Ready for Mission Segments.\n")

def execute_drive(bridge, distance_m):
    """Executes a straight drive for a specific distance."""
    print(f"[Ground] DRIVE: {distance_m}m at {SPEED_MPS}m/s")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, SPEED_MPS, 0, 0, 0, 0, 0, 0, 0)
    
    start_t = time.time()
    duration = distance_m / SPEED_MPS
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)
    
    # Stop
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)

def execute_turn(bridge, degrees):
    """Executes a pivot turn using yaw rate (Robust for Skid-Steer)."""
    print(f"[Ground] TURN: {degrees} degrees")
    yaw_rate = 45 if degrees > 0 else -45  # Increased rate for skid-steer torque
    duration = abs(degrees) / 45.0
    
    # 0x05C7 (0b010111000111) -> Ignore Pos, Acc, Yaw. Active: Velocity and Yaw Rate.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0x05C7,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        math.radians(yaw_rate))
    
    start_t = time.time()
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)
        
    # Stop rotation
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0x05C7,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)

def main():
    print(f"[Ground] Starting Bridge on {ESP32_BRIDGE_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="Ground-Station")
    try:
        bridge.connect()
        bridge.send_message("hi flying rat . we connected")
    except Exception as e:
        print(f"!!! Bridge Error: {e} !!!")
        return

    print("[Ground] READY: WAITING FOR AIR COMMAND...")
    seq = 0
    try:
        while True:
            msg = bridge.get_message()
            if msg: print(f"\n>>> [AIR MESSAGE]: {msg}\n")
            
            # Broadcast Status (Heartbeat)
            if seq % 10 == 0:
                print(f"[Ground] Heartbeat... Mode: {vehicle.mode.name} | Armed: {vehicle.armed} | Fix: {vehicle.gps_0.fix_type}")
            broadcast_status(bridge, seq)
            seq += 1

            # Command Handling
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStopFlag = cmd
                print(f"!!! [RADIO] Received CMD {cmdVal} seq {cmdSeq} !!!")
                
                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    if not vehicle.armed: arm_and_move(bridge)
                    execute_drive(bridge, DIST_M)
                elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                    if not vehicle.armed: arm_and_move(bridge)
                    execute_drive(bridge, 0.61) # 2ft
                elif cmdVal == v2v_bridge.CMD_TURN_RIGHT:
                    if not vehicle.armed: arm_and_move(bridge)
                    execute_turn(bridge, 90)
                elif cmdVal == v2v_bridge.CMD_TURN_LEFT:
                    if not vehicle.armed: arm_and_move(bridge)
                    execute_turn(bridge, -90)
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
