"""
tracker.py - Overhead Camera Tracker for Robot Swarm Localisation
Detects ArUco markers (4x4_50 dictionary) from a 1920x1080 camera feed,
converts pixel positions to real-world coordinates centred at (0, 0),
and pushes (x, y, theta) data to the UDP server.
"""

import cv2
import numpy as np
import math
import time
import platform
from threading import Thread
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

# ── Safety Configuration ───────────────────────────────────────────────────────
LOST_TIMEOUT_S        = 1.0    # seconds before a missing robot triggers STOP
COLLISION_DIST_M      = 0.20   # metres — stop if any two robots are closer than this
# ─────────────────────────────────────────────────────────────────────────────

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

    def start(self):
        t = Thread(target=self._update, daemon=True)
        t.start()
        return self

    def _update(self):
        frame_delta = 1.0 / FPS
        while not self.stopped:
            prev = time.time()
            self.grabbed, self.frame = self.stream.read()
            sleep = frame_delta - (time.time() - prev)
            if sleep > 0:
                time.sleep(sleep)

    def stop(self):
        self.stopped = True
        self.stream.release()


class CameraTracker:
    def __init__(self, server: UDPServer, active_robot_ids: list[int]):
        self.server = server
        self.active_robot_ids = set(active_robot_ids)

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

        # ── Safety state ───────────────────────────────────────────────────────
        # last_seen: robot_id → timestamp of most recent detection
        # Seeded to now so robots don't immediately trip the timeout on startup.
        now = time.time()
        self.last_seen: dict[int, float] = {rid: now for rid in active_robot_ids}
        self.safety_stop_active = False   # True while a safety condition holds

    def process_frame(self, frame: np.ndarray) -> dict[int, tuple[float, float, float]]:
        """
        Detect ArUco markers in frame and return position data for active robots.
        Returns: { robot_id: (x_m, y_m, theta_rad), ... }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        positions = {}
        if ids is None:
            return positions

        for corner, marker_id in zip(corners, ids.flatten()):
            if marker_id not in self.active_robot_ids:
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

        # ── Safety overlay ────────────────────────────────────────────────────
        now = time.time()

        # Highlight robots that have exceeded the lost timeout
        warn_row = 0
        for robot_id, last_t in self.last_seen.items():
            elapsed = now - last_t
            if elapsed >= LOST_TIMEOUT_S:
                warn_y = 30 + warn_row * 28
                cv2.putText(annotated,
                            f"LOST: Robot {robot_id} ({elapsed:.1f}s)",
                            (10, warn_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)
                warn_row += 1

        # Highlight close pairs with a line between them
        detected = list(positions.items())
        for i in range(len(detected)):
            id_a, (xa, ya, _) = detected[i]
            for j in range(i + 1, len(detected)):
                id_b, (xb, yb, _) = detected[j]
                dist = math.hypot(xa - xb, ya - yb)
                if dist < COLLISION_DIST_M:
                    pxa = int(CX + xa * PX_PER_M_X)
                    pya = int(CY - ya * PX_PER_M_Y)
                    pxb = int(CX + xb * PX_PER_M_X)
                    pyb = int(CY - yb * PX_PER_M_Y)
                    cv2.line(annotated, (pxa, pya), (pxb, pyb), (0, 0, 255), 3)
                    mid_x = (pxa + pxb) // 2
                    mid_y = (pya + pyb) // 2
                    cv2.putText(annotated, f"{dist:.2f}m!", (mid_x, mid_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Global safety banner
        if self.safety_stop_active:
            cv2.rectangle(annotated, (0, FRAME_HEIGHT - 50),
                          (FRAME_WIDTH, FRAME_HEIGHT), (0, 0, 180), -1)
            cv2.putText(annotated, "!! SAFETY STOP ACTIVE !!",
                        (FRAME_WIDTH // 2 - 280, FRAME_HEIGHT - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)

        return annotated

    def check_safety(
        self,
        positions: dict[int, tuple[float, float, float]],
    ) -> tuple[bool, str]:
        """
        Evaluate safety conditions and return (stop_required, reason_string).

        Conditions checked:
          1. Lost robot  — any active robot unseen for >= LOST_TIMEOUT_S seconds.
          2. Collision   — any two detected robots are within COLLISION_DIST_M metres.
        """
        now = time.time()

        # Update last-seen timestamps for every robot detected this frame.
        for robot_id in positions:
            self.last_seen[robot_id] = now

        # ── 1. Lost-robot check ───────────────────────────────────────────────
        for robot_id, last_t in self.last_seen.items():
            elapsed = now - last_t
            if elapsed >= LOST_TIMEOUT_S:
                return True, (
                    f"Robot {robot_id} not detected for {elapsed:.1f}s "
                    f"(threshold {LOST_TIMEOUT_S}s)"
                )

        # ── 2. Collision-proximity check ──────────────────────────────────────
        detected = list(positions.items())
        for i in range(len(detected)):
            id_a, (xa, ya, _) = detected[i]
            for j in range(i + 1, len(detected)):
                id_b, (xb, yb, _) = detected[j]
                dist = math.hypot(xa - xb, ya - yb)
                if dist < COLLISION_DIST_M:
                    return True, (
                        f"Robots {id_a} and {id_b} are {dist:.3f}m apart "
                        f"(threshold {COLLISION_DIST_M}m)"
                    )

        return False, ""

    def run(self):
        """Main tracking loop — processes frames and pushes data to the UDP server."""
        print("[Tracker] Starting tracking loop. Press 'q' in the video window to quit.")
        cv2.namedWindow("Robot Tracker", cv2.WINDOW_NORMAL)

        while True:
            if not self.vs.grabbed:
                print("[Tracker] Failed to read frame.")
                break

            frame = self.vs.frame.copy()
            positions = self.process_frame(frame)

            if positions:
                self.server.update_positions(positions)

            # ── Safety evaluation ─────────────────────────────────────────────
            stop_required, reason = self.check_safety(positions)

            if stop_required and not self.safety_stop_active:
                self.safety_stop_active = True
                self.server.set_command(run=False)
                print(f"[Safety] STOP triggered — {reason}")

            elif not stop_required and self.safety_stop_active:
                self.safety_stop_active = False
                self.server.set_command(run=True)
                print("[Safety] All conditions cleared — resuming RUN")

            annotated = self.annotate_frame(frame, positions)
            cv2.imshow("Robot Tracker", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[Tracker] Quit signal received.")
                break

        self.vs.stop()
        cv2.destroyAllWindows()
        print("[Tracker] Camera released.")


# ── Standalone test (no server required) ─────────────────────────────────────
if __name__ == "__main__":
    class StubServer:
        def update_positions(self, pos):
            print(f"[StubServer] Positions: {pos}")
        def set_command(self, run):
            print(f"[StubServer] Command: {'RUN' if run else 'STOP'}")

    tracker = CameraTracker(
        server=StubServer(),
        active_robot_ids=[0, 1, 2, 3, 4],
    )
    tracker.run()
