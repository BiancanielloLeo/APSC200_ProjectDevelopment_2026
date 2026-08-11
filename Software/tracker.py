"""
tracker.py - Overhead Camera Tracker for Robot Swarm Localisation
Detects ArUco markers (4x4_50 dictionary) from a 1920x1080 camera feed,
converts pixel positions to real-world coordinates centred at (0, 0),
and pushes (x, y, theta) data to the UDP server immediately after each frame.

KEY CHANGES:
- Packets sent immediately after frame processing (not on timer)
- GUI runs in separate thread (non-blocking)
- Frame rate and packet rate tracked for display
- Stops sending if camera feed is lost
"""

import cv2
import numpy as np
import math
import time
import platform
from threading import Thread
from collections import deque
from udp import UDPServer

# ── Camera / Scene Configuration ──────────────────────────────────────────────
CAMERA_INDEX      = 1          # OpenCV camera index
FRAME_WIDTH       = 1920       # Camera feed width  (pixels)
FRAME_HEIGHT      = 1080       # Camera feed height (pixels)
FPS               = 30

# Physical dimensions of the area visible to the camera (metres).
# Measure the floor region that fills the camera frame at the mounted height.
SCENE_WIDTH_M     = 3.6        # Real-world width  the camera sees (metres)
SCENE_HEIGHT_M    = 2.03       # Real-world height the camera sees (metres)
#   Pixel → metre scale factors
PX_PER_M_X = FRAME_WIDTH  / SCENE_WIDTH_M    # pixels per metre (horizontal)
PX_PER_M_Y = FRAME_HEIGHT / SCENE_HEIGHT_M   # pixels per metre (vertical)

# ArUco marker side length in metres (used for pose estimation).
MARKER_LENGTH_M   = 0.05       # 5 cm markers

# Centre pixel (maps to world origin (0, 0))
CX = FRAME_WIDTH  // 2         # 960
CY = FRAME_HEIGHT // 2         # 540
# ─────────────────────────────────────────────────────────────────────────────


def pixel_to_world(px: float, py: float) -> tuple[float, float]:
    """
    Convert pixel coordinates to world coordinates in metres.
    Centre pixel (CX, CY) maps to (0.0, 0.0).
    X increases rightward; Y increases upward (inverted from image convention).
    """
    x_m = (px - CX) / PX_PER_M_X
    y_m = -(py - CY) / PX_PER_M_Y   # invert Y so up = positive
    return x_m, y_m


def marker_heading(corners) -> float:
    """
    Compute heading from the top edge of the printed ArUco marker.
    'Top' is the edge between corner 0 (TL) and corner 1 (TR) as the
    marker appears when printed and viewed on screen.
    Returns angle in radians in [-pi, pi], cartesian convention (CCW positive).
    """
    pts = corners[0]                        # TL, TR, BR, BL
    top_mid    = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[3] + pts[2]) / 2.0
    dx =  (top_mid[0] - bottom_mid[0])
    dy = -(top_mid[1] - bottom_mid[1])     # flip Y for cartesian
    return math.atan2(dy, dx)


class WebcamVideoStream:
    """
    Threaded camera capture — opens the C922 at full 1920x1080 on Windows
    using CAP_DSHOW + MJPG, which is the only reliable way to get 1080p
    out of this camera via OpenCV on Windows.
    """
    def __init__(self, src=1, width=1920, height=1080, fps=30, focus=0):
        if platform.system() == "Windows":
            self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            self.stream = cv2.VideoCapture(src)

        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS,          fps)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        self.stream.set(cv2.CAP_PROP_AUTOFOCUS,    0)
        self.stream.set(cv2.CAP_PROP_FOCUS,        focus)

        self.stopped  = False
        self.grabbed, self.frame = self.stream.read()
        self.frame_count = 0  # NEW: track frame number

    def start(self):
        t = Thread(target=self._update, daemon=True)
        t.start()
        return self

    def _update(self):
        frame_delta = 1.0 / 30  # Read at camera's actual rate
        while not self.stopped:
            prev = time.time()
            self.grabbed, self.frame = self.stream.read()
            self.frame_count += 1  # NEW: increment on each new frame
            sleep = frame_delta - (time.time() - prev)
            if sleep > 0:
                time.sleep(sleep)

    def stop(self):
        self.stopped = True
        self.stream.release()


class FrameRateCounter:
    """Tracks frame/packet rate over a rolling window."""
    def __init__(self, window_size=30):
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)
    
    def tick(self):
        """Record a timestamp."""
        self.timestamps.append(time.time())
    
    def get_rate(self) -> float:
        """Return rate in Hz (events per second)."""
        if len(self.timestamps) < 2:
            return 0.0
        elapsed = self.timestamps[-1] - self.timestamps[0]
        if elapsed < 0.01:  # avoid division by very small numbers
            return 0.0
        return (len(self.timestamps) - 1) / elapsed


class CameraTracker:
    def __init__(self, server: UDPServer, active_robot_ids: list[int] = None):
        self.server = server
        # If active_robot_ids is None, detect all markers; otherwise use provided list
        self.active_robot_ids = set(active_robot_ids) if active_robot_ids is not None else None

        # ArUco setup
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector     = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera — threaded stream for reliable 1080p on the C922
        self.vs = WebcamVideoStream(
            src=CAMERA_INDEX,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=FPS,
            focus=0,
        ).start()

        if not self.vs.grabbed:
            raise RuntimeError(f"[Tracker] Cannot open camera index {CAMERA_INDEX}")
        print(f"[Tracker] Camera opened ({FRAME_WIDTH}x{FRAME_HEIGHT})")

        # Performance tracking
        self.frame_rate_counter = FrameRateCounter(window_size=30)
        self.packet_rate_counter = FrameRateCounter(window_size=30)
        
        # GUI state (shared between main thread and GUI thread)
        self.latest_frame = None
        self.latest_positions = {}
        self.gui_stopped = False

    def process_frame(self, frame: np.ndarray) -> dict[int, tuple[float, float, float]]:
        """
        Detect ArUco markers in frame and return position data.
        If active_robot_ids is None, detects all markers. Otherwise, filters by active_robot_ids.
        Returns: { robot_id: (x_m, y_m, theta_rad), ... }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        positions = {}
        if ids is None:
            return positions

        for corner, marker_id in zip(corners, ids.flatten()):
            # If active_robot_ids is None, detect all markers; otherwise filter
            if self.active_robot_ids is not None and marker_id not in self.active_robot_ids:
                continue

            pts = corner[0]
            cx_px = float(np.mean(pts[:, 0]))
            cy_px = float(np.mean(pts[:, 1]))
            x_m, y_m = pixel_to_world(cx_px, cy_px)
            theta = marker_heading(corner)

            positions[int(marker_id)] = (x_m, y_m, theta)

        return positions

    def annotate_frame(
        self,
        frame: np.ndarray,
        positions: dict[int, tuple[float, float, float]],
    ) -> np.ndarray:
        """Draw detections and the world-origin crosshair onto the frame."""
        annotated = frame.copy()

        # World origin crosshair
        cross_size = 30
        cv2.line(annotated, (CX - cross_size, CY), (CX + cross_size, CY), (0, 255, 0), 2)
        cv2.line(annotated, (CX, CY - cross_size), (CX, CY + cross_size), (0, 255, 0), 2)
        cv2.putText(annotated, "(0,0)", (CX + 8, CY - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Annotate each detected robot
        for robot_id, (x_m, y_m, theta) in positions.items():
            px = int(CX + x_m * PX_PER_M_X)
            py = int(CY - y_m * PX_PER_M_Y)

            # Heading arrow — points in the direction the front of the robot faces
            arrow_len = 40
            ex = int(px + arrow_len * math.cos(theta))
            ey = int(py - arrow_len * math.sin(theta))
            cv2.arrowedLine(annotated, (px, py), (ex, ey), (0, 0, 255), 2, tipLength=0.3)

            label = f"ID:{robot_id}  ({x_m:.2f}m, {y_m:.2f}m)  {math.degrees(theta):.1f}deg"
            cv2.putText(annotated, label, (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.circle(annotated, (px, py), 6, (255, 255, 0), -1)

        # Add performance metrics
        frame_rate = self.frame_rate_counter.get_rate()
        packet_rate = self.packet_rate_counter.get_rate()
        metrics_text = f"Frame Rate: {frame_rate:.1f} Hz | Packet Rate: {packet_rate:.1f} Hz"
        cv2.putText(annotated, metrics_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return annotated

    def gui_thread(self):
        """
        Runs in a background thread.
        Displays the annotated frame without blocking the main tracking loop.
        """
        cv2.namedWindow("Robot Tracker", cv2.WINDOW_NORMAL)
        
        while not self.gui_stopped:
            if self.latest_frame is not None:
                annotated = self.annotate_frame(self.latest_frame, self.latest_positions)
                cv2.imshow("Robot Tracker", annotated)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[Tracker] Quit signal received.")
                    self.gui_stopped = True
            else:
                # No frame yet, wait a bit
                time.sleep(0.01)
        
        cv2.destroyAllWindows()
        print("[Tracker] GUI thread stopped.")

    def run(self):
        """
        Main tracking loop — processes frames and sends packets immediately.
        GUI runs in a separate thread and doesn't block packet transmission.
        """
        print("[Tracker] Starting tracking loop. Press 'q' in the video window to quit.")
        
        # Start GUI in background thread
        gui_t = Thread(target=self.gui_thread, daemon=True)
        gui_t.start()

        last_frame_count = -1  # Track the frame count we last processed

        while True:
            # Check if camera is still connected
            if not self.vs.grabbed:
                print("[Tracker] Camera feed lost. Stopping packet transmission.")
                self.server.stop_server()  # Stop sending packets if camera is dead
                break

            # Get the latest frame
            frame = self.vs.frame.copy()
            
            # Wait for camera thread to provide a NEW frame
            if self.vs.frame_count == last_frame_count:
                # Same frame count = no new frame from camera yet
                time.sleep(0.001)  # Brief sleep, try again
                continue
            
            # New frame! Update our counter
            last_frame_count = self.vs.frame_count
            
            # Process frame to extract positions
            positions = self.process_frame(frame)
            self.frame_rate_counter.tick()

            # Send packet IMMEDIATELY after processing (not on a timer)
            if positions:
                self.server.send_packet_now(positions)
                self.packet_rate_counter.tick()

            # Store frame for GUI thread (non-blocking)
            self.latest_frame = frame.copy()
            self.latest_positions = positions

            # Check if GUI requested quit
            if self.gui_stopped:
                print("[Tracker] GUI quit requested.")
                break

        self.vs.stop()
        self.gui_stopped = True
        print("[Tracker] Camera released.")


# ── Standalone test (no server required) ─────────────────────────────────────
if __name__ == "__main__":
    class StubServer:
        def send_packet_now(self, pos):
            print(f"[StubServer] Sending packet: {pos}")
        def stop_server(self):
            pass

    tracker = CameraTracker(
        server=StubServer(),
        active_robot_ids=[0, 1, 2, 3, 4],
    )
    tracker.run()
