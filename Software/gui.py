"""
gui.py - GUI Display Module for Robot Tracker
Handles all visualization (frame display, annotations, metrics).
Runs in separate thread to avoid blocking packet transmission.

ARCHITECTURE:
  - Main thread (tracker.py): Processes frames, sends packets
  - GUI thread (gui.py): Displays frames, handles keyboard input
  - Communication: Shared attributes (latest_frame, latest_positions, metrics)
"""

import cv2
import math
from threading import Thread


class TrackerGUI:
    """
    OpenCV-based GUI for camera tracker visualization.
    Runs in a separate thread - does not block frame processing or packet transmission.
    """

    def __init__(self, frame_width: int = 1920, frame_height: int = 1080, 
                 center_x: int = 960, center_y: int = 540,
                 px_per_m_x: float = 533.33, px_per_m_y: float = 532.51):
        """
        Initialize GUI with camera parameters for coordinate transformation.
        
        Args:
            frame_width: Camera frame width in pixels
            frame_height: Camera frame height in pixels
            center_x: X coordinate of world origin (0,0) in pixels
            center_y: Y coordinate of world origin (0,0) in pixels
            px_per_m_x: Pixels per meter (X axis)
            px_per_m_y: Pixels per meter (Y axis)
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = center_x
        self.center_y = center_y
        self.px_per_m_x = px_per_m_x
        self.px_per_m_y = px_per_m_y
        
        # State from tracker thread
        self.latest_frame = None
        self.latest_positions = {}
        self.frame_rate = 0.0
        self.packet_rate = 0.0
        
        # Control
        self.stopped = False
        self.quit_requested = False

    def start(self):
        """Start GUI in background thread."""
        gui_thread = Thread(target=self._run, daemon=True)
        gui_thread.start()
        print("[GUI] Started in background thread")
        return self

    def stop(self):
        """Stop GUI thread gracefully."""
        self.stopped = True
        print("[GUI] Stop requested")

    def update(self, frame, positions: dict, frame_rate: float, packet_rate: float):
        """
        Update GUI with latest frame and data.
        Called from main tracker thread - this is non-blocking.
        
        Args:
            frame: Latest camera frame
            positions: Robot positions {robot_id: (x, y, theta), ...}
            frame_rate: Current frame rate in Hz
            packet_rate: Current packet rate in Hz
        """
        self.latest_frame = frame.copy() if frame is not None else None
        self.latest_positions = positions.copy()
        self.frame_rate = frame_rate
        self.packet_rate = packet_rate

    def _run(self):
        """Main GUI loop (runs in background thread)."""
        cv2.namedWindow("Robot Tracker", cv2.WINDOW_NORMAL)
        
        while not self.stopped:
            if self.latest_frame is not None:
                # Annotate frame
                annotated = self._annotate_frame(self.latest_frame, self.latest_positions)
                
                # Display
                cv2.imshow("Robot Tracker", annotated)
                
                # Handle keyboard input (non-blocking)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[GUI] Quit requested by user")
                    self.quit_requested = True
                    break
            else:
                # No frame yet, wait a bit
                cv2.waitKey(10)
        
        cv2.destroyAllWindows()
        print("[GUI] Closed")

    def _annotate_frame(self, frame, positions: dict):
        """
        Draw detections and metrics onto the frame.
        
        Args:
            frame: Original camera frame
            positions: Robot positions {robot_id: (x, y, theta), ...}
        
        Returns:
            Annotated frame with all overlays
        """
        annotated = frame.copy()
        
        # Draw world origin crosshair
        self._draw_origin_marker(annotated)
        
        # Draw each detected robot
        for robot_id, (x_m, y_m, theta) in positions.items():
            self._draw_robot(annotated, robot_id, x_m, y_m, theta)
        
        # Draw performance metrics
        self._draw_metrics(annotated)
        
        return annotated

    def _draw_origin_marker(self, frame):
        """Draw crosshair at world origin (0, 0)."""
        cross_size = 30
        color = (0, 255, 0)  # Green
        thickness = 2
        
        # Horizontal line
        cv2.line(frame, (self.center_x - cross_size, self.center_y), 
                (self.center_x + cross_size, self.center_y), color, thickness)
        
        # Vertical line
        cv2.line(frame, (self.center_x, self.center_y - cross_size), 
                (self.center_x, self.center_y + cross_size), color, thickness)
        
        # Label
        cv2.putText(frame, "(0,0)", (self.center_x + 8, self.center_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def _draw_robot(self, frame, robot_id: int, x_m: float, y_m: float, theta: float):
        """
        Draw robot marker, heading arrow, and label.
        
        Args:
            frame: Frame to draw on
            robot_id: Robot identifier
            x_m: X position in metres
            y_m: Y position in metres
            theta: Heading angle in radians
        """
        # Convert world coordinates to pixel coordinates
        px = int(self.center_x + x_m * self.px_per_m_x)
        py = int(self.center_y - y_m * self.px_per_m_y)
        
        # Draw robot position circle
        cv2.circle(frame, (px, py), 6, (255, 255, 0), -1)  # Yellow filled circle
        
        # Draw heading arrow
        arrow_len = 40
        ex = int(px + arrow_len * math.cos(theta))
        ey = int(py - arrow_len * math.sin(theta))
        cv2.arrowedLine(frame, (px, py), (ex, ey), (0, 0, 255), 2, tipLength=0.3)  # Red arrow
        
        # Draw label with position and heading
        label = f"ID:{robot_id}  ({x_m:.2f}m, {y_m:.2f}m)  {math.degrees(theta):.1f}deg"
        cv2.putText(frame, label, (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)  # Yellow text

    def _draw_metrics(self, frame):
        """Draw real-time performance metrics."""
        metrics_text = f"Frame Rate: {self.frame_rate:.1f} Hz | Packet Rate: {self.packet_rate:.1f} Hz"
        cv2.putText(frame, metrics_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)  # White text

    def was_quit_requested(self) -> bool:
        """Check if user pressed 'q' to quit."""
        return self.quit_requested


# ── Standalone test ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    import time
    import numpy as np
    
    # Create test frame
    test_frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    test_frame[:] = (50, 50, 50)  # Dark gray background
    
    # Create GUI
    gui = TrackerGUI()
    gui.start()
    
    print("[Test] GUI started. Displaying test data for 5 seconds...")
    print("[Test] Press 'q' in the window to quit early")
    
    # Simulate tracker data
    for i in range(150):  # 5 seconds at 30 FPS
        test_positions = {
            1: (0.5 + 0.1 * math.sin(i * 0.1), 0.3, i * 0.05),
            2: (-0.3, -0.5 + 0.15 * math.cos(i * 0.08), -i * 0.03),
            3: (0.0, 0.5, 0.0),
        }
        
        gui.update(test_frame, test_positions, 30.0, 29.5)
        
        if gui.was_quit_requested():
            print("[Test] User pressed 'q', exiting...")
            break
        
        time.sleep(1/30)  # 30 FPS
    
    gui.stop()
    print("[Test] Done")
