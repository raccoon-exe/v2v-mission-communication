from pymavlink import mavutil
import time
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
CONNECTION_STRING = "/dev/ttyACM0" 
BAUD_RATE = 115200
ESP32_PORT = "/dev/ttyUSB0"

# ------------------- Connect -------------------
def main():
    print("==========================================")
    print("   UAV MISSION 1 - GROUND COORDINATION")
    print("==========================================")
    
    print(f"[Mission 1] Connecting to UAV Controller at {CONNECTION_STRING}...")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("[Mission 1] Drone Heartbeat found. Sensors Check: OK")
    except Exception as e:
        print(f"!!! Error connecting to Drone: {e} !!!")
        print("Continuing with Bridge only...")
        master = None

    # Initialize V2V Bridge
    print(f"[Mission 1] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        # Debug link check
        print("[Mission 1] Sending Hello to UGV...")
        bridge.send_message("hello pissrat . we connected")
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    # ------------------- Helpers -------------------
    def arm_drone():
        if master:
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 
                                         0, 0, 0, 0, 0, 0)
            print("[Mission 1] Drone Motors Engaged (Ground Only).")

    def disarm_drone():
        if master:
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 
                                         0, 0, 0, 0, 0, 0)
            print("[Mission 1] Drone Disarmed.")

    def broadcast_uav_status(seq):
        """Fetches drone status via MAVLink and sends it over radio."""
        armed_val = 0
        mode_val = v2v_bridge.MODE_INITIAL
        
        if master:
            # Request small status update
            # We look for HEARTBEAT to get mode and armed state
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
                # Mode extraction varies by flight stack, but simple INITIAL is fine for ground test
                # If we wanted specific modes, we'd map msg.custom_mode
        
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        bridge.send_telemetry(seq, t_ms, 0.0, 0.0, armed_val, mode_val)

    # ------------------- EXECUTION -------------------
    try:
        # 1. Arm Drone on Ground
        arm_drone()
        time.sleep(2)
        
        # 2. WAIT FOR UGV SYNC:
        print("[Mission 1] Waiting for UGV Status Sync via Radio...")
        ugv_ready = False
        timeout_start = time.time()
        
        while not ugv_ready:
            # 1. READ DEBUG MESSAGES (Echoes from the ESP32)
            msg = bridge.get_message()
            if msg:
                print(f"    >>> [RADIO MSG]: {msg}")

            # 2. Check for Telemetry FROM the UGV
            data = bridge.get_telemetry()
            if data:
                seq_u, t_ms_u, vx_u, vy_u, armed_u, mode_u = data
                status_str = "ARMED" if armed_u == 1 else "DISARMED"
                mode_str = "GUIDED" if mode_u == v2v_bridge.MODE_GUIDED else "INITIAL"
                print(f"    [RADIO] UGV STATUS: {status_str} | MODE: {mode_str}")
                
                # If we see ANY telemetry, the link is working!
                print("\n!!! [SYNC] UGV RADIO LINK VERIFIED !!!")
                ugv_ready = True
            
            # Send our own telemetry so UGV knows we are alive
            broadcast_uav_status(0)
            
            time.sleep(1.0)
            if time.time() - timeout_start > 30: # 30s timeout
                print("!!! [TIMEOUT] UGV did not arm. Aborting mission sequence.")
                break

        if ugv_ready:
            # 3. COORDINATED ACTION: Tell UGV to move!
            print("[Mission 1] >>> COMMANDING UGV TO MOVE 10ft FRONT")
            bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) 

            # 4. Simulation of "Moving" time
            print("[Mission 1] Tracking UGV progress...")
            for i in range(10):
                # Print real-time speed from UGV if available
                telem = bridge.get_telemetry()
                if telem:
                    v_mps = telem[2]
                    print(f"  Driving... Speed: {v_mps:.1f} m/s")
                time.sleep(1.0)

        # 5. Clean up
        print("[Mission 1] Test Sequence Complete.")
        disarm_drone()

    except KeyboardInterrupt:
        print("[Mission 1] User Interrupted. Safety Disarming...")
        disarm_drone()
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
