"""
udp.py - UDP Broadcast Server for Robot Swarm Localisation
Receives robot position data from the camera tracker and broadcasts
packets to all robots on the network.

KEY CHANGES:
- send_packet_now(positions) sends immediately when tracker calls it
- Optional background broadcast loop (for fallback if camera is slow)
- Stops sending if explicitly commanded
"""

import socket
import struct
import threading
import time

# ── Network Configuration ─────────────────────────────────────────────────────
WIFI_SSID     = "RobotWifi"             # Wi-Fi network name
WIFI_PASSWORD = "12345678"              # Wi-Fi password
SERVER_IP     = "0.0.0.0"               # Bind to all interfaces
SERVER_PORT   = 5005                    # UDP port
BROADCAST_IP  = "255.255.255.255"       # Broadcast to all devices on subnet
# ─────────────────────────────────────────────────────────────────────────────

# Packet format (per robot entry):
#   command      : 1 byte  ('R' = run, 'S' = stop)
#   packet_number: 4 bytes (unsigned int, big-endian)
#   num_robots   : 1 byte
#   For each robot:
#     robot_id   : 1 byte
#     x          : 4 bytes (float, metres)
#     y          : 4 bytes (float, metres)
#     theta      : 4 bytes (float, radians)

HEADER_FORMAT  = "!cIB"          # command, packet_number, num_robots
ROBOT_FORMAT   = "!Bfff"        # robot_id, x, y, theta
HEADER_SIZE    = struct.calcsize(HEADER_FORMAT)
ROBOT_ENTRY_SIZE = struct.calcsize(ROBOT_FORMAT)


class UDPServer:
    def __init__(self):
        self.robot_positions: dict[int, tuple[float, float, float]] = {}
        self.lock = threading.Lock()
        self.packet_number = 0
        self.running = True          # True = send RUN command; False = send STOP
        self.active = True           # Server loop control
        self.background_broadcast_enabled = False  # Background thread control

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((SERVER_IP, SERVER_PORT))
        print(f"[UDP] Server bound to {SERVER_IP}:{SERVER_PORT}")
        print(f"[UDP] Broadcasting to {BROADCAST_IP}:{SERVER_PORT}")

    # ── Called by the camera tracker to push updated positions ────────────────
    def update_positions(self, positions: dict[int, tuple[float, float, float]]):
        """
        Update the robot position table (used by background broadcast loop).
        positions: { robot_id: (x, y, theta), ... }
        """
        with self.lock:
            self.robot_positions.update(positions)

    def send_packet_now(self, positions: dict[int, tuple[float, float, float]]):
        """
        Send a packet IMMEDIATELY with the given positions.
        This is called directly by the tracker after processing each frame.
        positions: { robot_id: (x, y, theta), ... }
        """
        with self.lock:
            # REPLACE positions with current frame data (don't merge with old data)
            self.robot_positions = positions.copy()
            packet = self._build_packet()
        
        try:
            self.sock.sendto(packet, (BROADCAST_IP, SERVER_PORT))
        except OSError as e:
            print(f"[UDP] Error sending packet: {e}")

    def set_command(self, run: bool):
        """Switch between RUN and STOP broadcast command."""
        self.running = run
        state = "RUN" if run else "STOP"
        print(f"[UDP] Command set to: {state}")

    def stop_server(self):
        """Gracefully shut down the broadcast loop."""
        self.active = False
        try:
            self.sock.close()
        except OSError:
            pass
        print("[UDP] Server stopped.")

    # ── Packet construction ───────────────────────────────────────────────────
    def _build_packet(self) -> bytes:
        """
        Build a UDP packet with current state.
        MUST be called with self.lock held!
        """
        command = b'R' if self.running else b'S'
        self.packet_number = (self.packet_number + 1) % (2**32)
        # Sort robots by ID for deterministic packet structure
        robots = sorted(self.robot_positions.items(), key=lambda x: x[0])

        num_robots = len(robots)
        header = struct.pack(HEADER_FORMAT, command, self.packet_number, num_robots)
        body = b""
        for robot_id, (x, y, theta) in robots:
            body += struct.pack(ROBOT_FORMAT, robot_id, x, y, theta)
        return header + body

    # ── Background broadcast loop (optional fallback) ─────────────────────────
    def broadcast_loop(self, rate_hz: float = 20.0):
        """
        Continuously broadcast position packets at the given rate (default 20 Hz).
        This is only needed if the camera/tracker is slower than desired.
        If tracker is running at ~30 Hz, you typically won't need this.
        """
        interval = 1.0 / rate_hz
        print(f"[UDP] Background broadcast loop started at {rate_hz} Hz...")
        while self.active:
            try:
                with self.lock:
                    if self.robot_positions:
                        packet = self._build_packet()
                    else:
                        # No robots yet, skip this interval
                        time.sleep(interval)
                        continue
                
                self.sock.sendto(packet, (BROADCAST_IP, SERVER_PORT))
            except OSError:
                break
            time.sleep(interval)

    def start(self, rate_hz: float = 20.0, enable_background_broadcast: bool = False):
        """
        Start the server.
        If enable_background_broadcast=True, also start a fallback broadcast loop.
        """
        if enable_background_broadcast:
            self.background_broadcast_enabled = True
            t = threading.Thread(target=self.broadcast_loop, args=(rate_hz,), daemon=True)
            t.start()
        return self


# ── Packet decoder (for debugging / robot-side reference) ────────────────────
def decode_packet(data: bytes):
    """Parse a raw UDP packet and return (command, packet_number, robots_dict)."""
    command_byte, packet_number, num_robots = struct.unpack_from(HEADER_FORMAT, data, 0)
    command = "RUN" if command_byte == b'R' else "STOP"
    robots = {}
    offset = HEADER_SIZE
    for _ in range(num_robots):
        robot_id, x, y, theta = struct.unpack_from(ROBOT_FORMAT, data, offset)
        robots[robot_id] = (x, y, theta)
        offset += ROBOT_ENTRY_SIZE
    return command, packet_number, robots


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    server = UDPServer()
    server.set_command(run=True)
    server.start(rate_hz=10, enable_background_broadcast=True)

    # Inject dummy data and send immediately
    dummy_positions = {
        1: (0.10, 0.20, 0.0),
        2: (0.50, -0.30, 1.57),
    }
    
    print("\n[Test] Sending 5 packets immediately...")
    for i in range(5):
        server.send_packet_now(dummy_positions)
        time.sleep(0.1)

    print("[Test] Packets sent. Waiting 2 seconds before shutdown...")
    time.sleep(2)
    server.stop_server()
