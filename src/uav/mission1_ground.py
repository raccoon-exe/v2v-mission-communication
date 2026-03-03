import time
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
# Since you are on Windows, check Device Manager for COM ports!
# Example: ESP32_PORT = "COM16"
ESP32_PORT = "COM16" 

# ------------------- Start Bridge -------------------
def main():
    print(f"[Mission 1] Starting V2V Bridge on {ESP32_PORT}...")
    
    # 1. Start the Communication Bridge!
    # We don't need the Cube Orange for this Ground Test.
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        print("Make sure your ESP32 is plugged in and the COM port is correct in the script.")
        return

    print("[Mission 1] Bridge Started. Sending 'MOVE' command to UGV...")

    # 2. COORDINATED ACTION: Tell UGV to move!
    # We send it a few times to make sure it gets through the radio link
    for i in range(5):
        print(f"Sending Command {i+1}/5...")
        bridge.send_command(cmdSeq=i+1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) 
        time.sleep(0.5)

    print("[Mission 1] Command Sent. You can check the UGV Pi now.")
    
    # 3. Keep sending heartbeat telemetry so the ground side sees us 'Live'
    print("Press Ctrl+C to stop heartbeats.")
    telem_seq = 0
    try:
        while True:
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            bridge.send_telemetry(telem_seq, t_ms, 0.0, 0.0, 0, 0)
            telem_seq += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping Mission 1.")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()
