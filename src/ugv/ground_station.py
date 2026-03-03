from dronekit import connect, VehicleMode
import time
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
UGV_CONTROL_PORT = "/dev/ttyACM1"  # UGV Flight Controller (Cube/Pixhawk)
ESP32_BRIDGE_PORT = "/dev/ttyACM0" # UGV ESP32 Bridge

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 0.5      # Ground speed

# ------------------- UGV Setup (DroneKit) -------------------
print(f"[Ground] Connecting to UGV Controller at {UGV_CONTROL_PORT}...")
try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=115200)
    print("[Ground] UGV Connected!")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    print("Check your COM port in the script.")
    exit()

def arm_and_move():
    """Arms the UGV and moves it forward 10ft."""
    print("[Ground] Checking if UGV is armable...")
    while not vehicle.is_armable:
        print("  Waiting for vehicle to become armable (check GPS/IMU)...")
        time.sleep(1)
    
    print("[Ground] Setting mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    
    print("[Ground] Arming UGV Motors...")
    vehicle.armed = True
    
    while not vehicle.armed:
        print("  Waiting for arming...")
        time.sleep(1)
    
    print("************************************")
    print("!!! UGV ARMED AND READY !!!")
    print("************************************")
    
    print(f"[Ground] UGV Moving Forward {DIST_M}m...")
    
    # Velocity command (forward)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        SPEED_MPS, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)

    start_t = time.time()
    duration = DIST_M / SPEED_MPS
    
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
    
    # Stop
    print("[Ground] Move Complete. Stopping and Disarming...")
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(stop_msg)
    
    vehicle.armed = False
    print("[Ground] Mission Complete.")

# ------------------- MAIN GROUND STATION -------------------
def main():
    # 1. Start the Radio Bridge
    print(f"[Ground] Starting Bridge on {ESP32_BRIDGE_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="Ground-Station")
    try:
        bridge.connect()
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    print("[Ground] Ready and Listening for Air Command...")
    
    try:
        while True:
            # 2. Monitor Air Telemetry
            telem = bridge.get_telemetry()
            if telem:
                seq, t_ms, vx, vy, marker, estop = telem
                # print(f"[TELEM] Air Status: Seq={seq} Estop={estop}")

            # 3. Handle Air Commands
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStopFlag = cmd
                
                # --- COMMAND: MOVE 10 FT ---
                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    print("!!! [AIR COMMAND] MOVE 10 FT FRONT RECEIVED !!!")
                    arm_and_move()

                # --- ABORT / E-STOP ---
                elif eStopFlag == 1:
                    print("!!! [ABORT] EMERGENCY ABORT RECEIVED !!!")
                    vehicle.mode = VehicleMode("LAND") # Or stop for rover
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("[Ground] Stopping...")
    finally:
        bridge.stop()
        vehicle.close()

if __name__ == "__main__":
    from pymavlink import mavutil # Needed for the velocity message
    main()
