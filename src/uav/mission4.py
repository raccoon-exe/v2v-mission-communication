from pymavlink import mavutil
import time
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
CONNECTION_STRING = "/dev/ttyACM0" 
BAUD_RATE = 115200
ESP32_PORT = "/dev/ttyUSB0"

def main():
    print("==========================================")
    print("   UAV MISSION 4 - CIRCLE MANEUVERS")
    print("==========================================")
    
    print(f"[Mission 4] Connecting to UAV Controller at {CONNECTION_STRING}...")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("[Mission 4] Drone Heartbeat found. Sensors Check: OK")
    except Exception as e:
        print(f"!!! Error connecting to Drone: {e} !!!")
        master = None

    print(f"[Mission 4] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("MISSION 4 STARTING: DOUBLE CIRCLE")
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    def broadcast_uav_status():
        armed_val = 0
        if master:
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        bridge.send_telemetry(0, t_ms, 0.0, 0.0, armed_val, 0)

    try:
        print("[Mission 4] Waiting for UGV Sync...")
        ugv_ready = False
        while not ugv_ready:
            data = bridge.get_telemetry()
            if data:
                seq_u, t_ms_u, v_u, _, armed_u, safety_byte = data
                is_armable = bool(safety_byte & 0x10)
                print(f"    [RADIO] UGV Sync: ARMED={armed_u}, Armable={is_armable}")
                ugv_ready = True
            broadcast_uav_status()
            time.sleep(1.0)

        # 2. COORDINATED SEQUENCE
        print("\n[Mission 4] >>> INITIATING DOUBLE CIRCLE (2x360)")
        
        # Trigger Circle Command
        # 1.0 m/s and 45 deg/s = 8s per circle. 2 circles = 16s.
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0)
        
        start_mission = time.time()
        # Wait for 2 circles (16s) + buffer (2s)
        total_duration = 18.0
        
        while (time.time() - start_mission) < total_duration:
            data = bridge.get_telemetry()
            if data:
                v_mps = data[2]
                elapsed = time.time() - start_mission
                print(f"  Driving Circle... T+{elapsed:.1f}s | Speed: {v_mps:.2f} m/s", end='\r')
            time.sleep(0.5)

        print("\n[Mission 4] CIRCLE SEQUENCE COMPLETE.")

    except KeyboardInterrupt:
        print("[Mission 4] Interrupted.")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
