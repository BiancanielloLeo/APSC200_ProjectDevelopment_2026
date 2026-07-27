"""
udp.py - UDP Broadcast Server for Robot Swarm Localisation
Receives robot position data from the camera tracker and broadcasts
packets to all robots on the network.
"""

import socket
import struct
import threading
import time

# ── Network Configuration ─────────────────────────────────────────────────────
WIFI_SSID     = "AgentWifi"        # Wi-Fi network name
WIFI_PASSWORD = "12345678"   # Wi-Fi password
SERVER_IP     = ""         # "192.168.1.148" zIP address this server binds to
SERVER_PORT   = 5005                    # UDP port
#BROADCAST_IP  = "192.168.1.255"
BROADCAST_IP  = "255.255.255.255"        # Subnet broadcast address
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

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((SERVER_IP, SERVER_PORT))
        print(f"[UDP] Server bound to {SERVER_IP}:{SERVER_PORT}")
        print(f"[UDP] Broadcasting to {BROADCAST_IP}:{SERVER_PORT}")

    # ── Called by the camera tracker to push updated positions ────────────────
    def update_positions(self, positions: dict[int, tuple[float, float, float]]):
        """
        Update the robot position table.
        positions: { robot_id: (x, y, theta), ... }
        """
        with self.lock:
            self.robot_positions.update(positions)

    def set_command(self, run: bool):
        """Switch between RUN and STOP broadcast command."""
        self.running = run
        state = "RUN" if run else "STOP"
        print(f"[UDP] Command set to: {state}")

    def stop_server(self):
        """Gracefully shut down the broadcast loop."""
        self.active = False
        self.sock.close()
        print("[UDP] Server stopped.")

    # ── Packet construction ───────────────────────────────────────────────────
    def _build_packet(self) -> bytes:
        with self.lock:
            command = b'R' if self.running else b'S'
            self.packet_number = (self.packet_number + 1) % (2**32)
            robots = list(self.robot_positions.items())

        num_robots = len(robots)
        header = struct.pack(HEADER_FORMAT, command, self.packet_number, num_robots)
        body = b""
        for robot_id, (x, y, theta) in robots:
            body += struct.pack(ROBOT_FORMAT, robot_id, x, y, theta)
        return header + body

    # ── Broadcast loop ────────────────────────────────────────────────────────
    def broadcast_loop(self, rate_hz: float = 20.0):
        """Continuously broadcast position packets at the given rate (default 20 Hz)."""
        interval = 1.0 / rate_hz
        print(f"[UDP] Broadcasting at {rate_hz} Hz...")
        while self.active:
            try:
                packet = self._build_packet()
                self.sock.sendto(packet, (BROADCAST_IP, SERVER_PORT))
            except OSError:
                break
            time.sleep(interval)

    def start(self, rate_hz: float = 20.0):
        """Start the broadcast loop in a background thread."""
        t = threading.Thread(target=self.broadcast_loop, args=(rate_hz,), daemon=True)
        t.start()
        return t


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

    # Inject dummy data for testing
    server.update_positions({
        1: (0.10, 0.20, 0.0),
        2: (0.50, -0.30, 1.57),
    })

    broadcast_thread = server.start(rate_hz=10)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[UDP] Keyboard interrupt — stopping.")
        server.stop_server()
