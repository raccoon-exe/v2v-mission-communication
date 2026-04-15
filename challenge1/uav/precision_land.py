"""
Challenge 1 - UAV Precision Landing on Moving UGV
=================================================
Automated takeoff + Precision Loiter + Precision Land using MAVLink LANDING_TARGET.
Uses the robust CameraInterface from the user's friend's code.
"""
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import os
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from pymavlink import mavutil

# ─── USER CONFIGURATION ────────────────────────────────────────────────────────
MAVLINK_CONN = "/dev/ttyACM0"      # Or udp:127.0.0.1:14550
BAUD_RATE    = 921600

# Nested ArUco Marker Setup
LARGE_MARKER_ID   = 5
SMALL_MARKER_ID   = 6
LARGE_MARKER_SIZE = 0.3048   # 12 inches -> meters
SMALL_MARKER_SIZE = 0.1016   # 4 inches -> meters

USE_SMALL_BELOW_M = 1.0
SEND_HZ = 15.0

# Flight Logic
TARGET_HEIGHT_M = 2.0
PLND_STABLE_FRAMES = 15   # Frames of solid tracking before engaging Precision Loiter
LAND_LATERAL_ERR_M = 0.20 # Meters of allowed lateral error before switching to LAND

# ─── FRIEND'S ROBUST CAMERA & ARUCO CODE ───────────────────────────────────────
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray

class CameraInterface:
    def __init__(self, use_zed: bool = False, camera_index: int = 0, width: int = 1280, height: int = 720, fps: int = 30):
        self.use_zed = use_zed
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.zed = None
        self.sl = None
        self.camera_matrix = None
        self.dist_coeffs = None
        if self.use_zed:
            self._open_zed()
        else:
            self._open_standard()

    def _open_zed(self):
        import pyzed.sl as sl
        self.sl = sl
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = self.fps
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array([[calib.fx, 0.0, calib.cx], [0.0, calib.fy, calib.cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5: self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0: self.dist_coeffs = dist.reshape(-1, 1)
        else: self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _open_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def load_standard_calibration(self, yaml_path: Optional[str]):
        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            K = fs.getNode("K").mat()
            D = fs.getNode("D").mat()
            fs.release()
            if K is not None and D is not None:
                self.camera_matrix = np.array(K, dtype=np.float64)
                self.dist_coeffs = np.array(D, dtype=np.float64)
                print(f"Loaded custom camera calibration from {yaml_path}")
            return

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS: return None
            image = self.sl.Mat()
            self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[-1] == 4: frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            else: frame = frame.copy()
            return frame
        else:
            ret, frame = self.cap.read()
            if not ret: return None
            return frame

    def close(self):
        if self.use_zed and self.zed is not None: self.zed.close()
        if self.cap is not None: self.cap.release()
        cv2.destroyAllWindows()

class ArucoDistanceEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, dictionary_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)
        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_detector_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_detector_api = False

    def detect_markers(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_detector_api: corners, ids, _ = self.detector.detectMarkers(gray)
        else: corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0: return poses

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i], marker_size_m)
            if rvec is None or tvec is None: continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(marker_id)] = MarkerPose(
                marker_id=int(marker_id), rvec=rvec, tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])), corners=corners[i][0]
            )
        return poses

    def _estimate_pose(self, corner: np.ndarray, marker_size_m: float):
        half = marker_size_m / 2.0
        object_points = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float32)
        image_points = corner.reshape((4, 2)).astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success: return None, None
        return rvec, tvec

def draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)

# ─── MAVLINK HELPERS ───────────────────────────────────────────────────────────
master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
print("Waiting for heartbeats from UAV...")
master.wait_heartbeat()
print("Heartbeat received!")

def send_guided_velocity(vx, vy, vz):
    """
    Directly force drone velocities in GUIDED mode (Body Frame).
    vx = Forward velocity (m/s)
    vy = Right velocity (m/s)
    vz = Down velocity (m/s)
    """
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, # bitmask: only use velocity
        0, 0, 0,            # x, y, z positions (ignored)
        vx, vy, vz,         # vx, vy, vz velocities
        0, 0, 0,            # x, y, z accelerations (ignored)
        0, 0                # yaw, yaw_rate (ignored)
    )

def change_mode(mode_name: str):
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None: return False
    master.set_mode(mode_id)
    return True

def get_rel_alt_m():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg: return msg.relative_alt / 1000.0
    return None

def arm_and_takeoff(alt):
    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed! Taking off...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt
    )

# ─── MAIN PROGRAM ──────────────────────────────────────────────────────────────
def main():
    # Attempt ZED first to prevent v4l2 OpenCV timeout crash
    USE_ZED = True  
    yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_chessboard.yaml")
    
    # Intentionally initialize camera flawlessly using Friend's script wrapper
    try:
        # Toggle USE_ZED=True if you prefer ZED natively
        cam = CameraInterface(use_zed=USE_ZED, camera_index=0, fps=30)
        cam.load_standard_calibration(yaml_path)
    except Exception as e:
        print(f"Failed to open camera: {e}")
        return

    estimator = ArucoDistanceEstimator(cam.camera_matrix, cam.dist_coeffs, aruco.DICT_6X6_1000)

    # STATE MACHINE
    state = "TAKEOFF"
    stable_count = 0
    last_send = 0.0

    print("Starting Automated Precision Landing Sequence...")
    print(">>> Drone will automatically Arm and Takeoff to 2.0m!")
    try:
        change_mode("GUIDED")
        arm_and_takeoff(TARGET_HEIGHT_M)
        state = "APPROACH"
        
        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            draw_crosshair(frame)

            # Detect all markers, using Large Marker size to start
            poses = estimator.detect_markers(frame, LARGE_MARKER_SIZE)
            
            found = False
            target_eb = None
            marker_id_to_use = None
            marker_size_m_to_use = None

            # Priority 1: Check if the Small Marker is visible
            if SMALL_MARKER_ID in poses:
                # Re-estimate properly with small marker size
                poses = estimator.detect_markers(frame, SMALL_MARKER_SIZE)
                if SMALL_MARKER_ID in poses:
                    marker_id_to_use = SMALL_MARKER_ID
                    marker_size_m_to_use = SMALL_MARKER_SIZE
            # Priority 2: Fallback to Large Marker
            elif LARGE_MARKER_ID in poses:
                marker_id_to_use = LARGE_MARKER_ID
                marker_size_m_to_use = LARGE_MARKER_SIZE

            if marker_id_to_use is not None and marker_id_to_use in poses:
                pose = poses[marker_id_to_use]
                # Convert ZED/Webcam standard frame XYZ to ArduPilot FRD body frame
                x_cam, y_cam, z_cam = float(pose.tvec[0]), float(pose.tvec[1]), float(pose.tvec[2])
                x_b = -y_cam  # Forward
                y_b = x_cam   # Right
                z_b = z_cam   # Down
                target_eb = (x_b, y_b, z_b)

                # 1. Visualization: Draw targeting line from camera center to marker center
                cam_cx, cam_cy = frame.shape[1] // 2, frame.shape[0] // 2
                marker_cx, marker_cy = pose.center_px
                cv2.line(frame, (cam_cx, cam_cy), (marker_cx, marker_cy), (0, 0, 255), 4)
                
                stable_count += 1
                found = True

                corners_arr = [pose.corners.reshape(1, 4, 2).astype(np.float32)]
                aruco.drawDetectedMarkers(frame, corners_arr, np.array([[marker_id_to_use]]))
                cv2.drawFrameAxes(frame, cam.camera_matrix, cam.dist_coeffs, pose.rvec, pose.tvec.reshape(3, 1), marker_size_m_to_use * 0.5)

                cv2.putText(frame, f"TRACKING ID: {marker_id_to_use}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"XYZ: [{x_b:.2f}, {y_b:.2f}, {z_b:.2f}]", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 2. Control Logic: Velocity P-Controller
                if state in ["APPROACH", "PREC_LOITER", "LANDING"]:
                    # Proportional Gain (how aggressive it chases)
                    Kp = 1.0  
                    vx = float(x_b) * Kp
                    vy = float(y_b) * Kp
                    vz = 0.0  # Default to holding altitude
                    
                    # Clamp velocities safely
                    max_speed = 1.5
                    vx = max(-max_speed, min(max_speed, vx))
                    vy = max(-max_speed, min(max_speed, vy))

                    now = time.time()
                    if now - last_send >= (1.0 / SEND_HZ):
                        if state == "LANDING":
                            vz = 0.5  # Slowly descend while aligning!
                        
                        send_guided_velocity(vx, vy, vz)
                        last_send = now

            else:
                stable_count = 0
                cv2.putText(frame, "TARGET LOST - HOVERING", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Halt the drone if it loses sight
                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ) and state in ["APPROACH", "PREC_LOITER"]:
                    send_guided_velocity(0.0, 0.0, 0.0)
                    last_send = now

            # STATE MACHINE (Transitions)
            if state == "APPROACH":
                if stable_count >= PLND_STABLE_FRAMES:
                    print(">>> Target stably held. Engaging GUIDED Velocity Lock!")
                    state = "PREC_LOITER"
            
            elif state == "PREC_LOITER":
                if stable_count == 0:
                    state = "APPROACH"
                elif found and target_eb is not None:
                    err_m = math.sqrt(target_eb[0]**2 + target_eb[1]**2)
                    cv2.putText(frame, f"Align Err: {err_m:.2f}m", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                    if err_m < LAND_LATERAL_ERR_M:
                        print(">>> Aligned perfectly! Forcing descend velocity!")
                        state = "LANDING"
            
            elif state == "LANDING":
                if not master.motors_armed():
                    print(">>> TOUCHDOWN DETECTED. DISARMED.")
                    state = "LANDED"
                    break
            
            cv2.putText(frame, f"MODE: {state}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.imshow("Precision Landing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("Cleaning up system...")
        cam.close()
        if state != "LANDED":
            change_mode("GUIDED")

if __name__ == "__main__":
    main()
