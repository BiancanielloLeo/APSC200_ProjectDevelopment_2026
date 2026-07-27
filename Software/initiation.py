"""
initiation.py - Swarm Initialisation Entry Point
Prompts the user for robot configuration, starts the UDP broadcast server,
launches the camera tracker, and provides a manual STOP command interface.
"""

import threading
import time
import sys
from udp import UDPServer
from tracker import CameraTracker


def get_robot_config() -> tuple[int, list[int]]:
    """Prompt the user for the number of robots and their IDs."""
    print("=" * 50)
    print("  Robot Swarm — Initiation")
    print("=" * 50)

    while True:
        try:
            num_robots = int(input("\nHow many robots are being used? "))
            if num_robots < 1:
                print("  Please enter at least 1.")
                continue
            break
        except ValueError:
            print("  Invalid input — enter a whole number.")

    print(f"\nEnter the ArUco marker ID for each of the {num_robots} robot(s).")
    print("(IDs must match the 4x4_50 dictionary markers attached to the robots.)\n")

    robot_ids = []
    for i in range(num_robots):
        while True:
            try:
                rid = int(input(f"  Robot {i + 1} marker ID: "))
                if rid in robot_ids:
                    print(f"  ID {rid} already used — choose a different one.")
                    continue
                robot_ids.append(rid)
                break
            except ValueError:
                print("  Invalid input — enter a whole number.")

    return num_robots, robot_ids


def print_status(robot_ids: list[int], server: UDPServer):
    """Display a summary of the active configuration."""
    print("\n" + "=" * 50)
    print("  Configuration Summary")
    print("=" * 50)
    print(f"  Active robots : {robot_ids}")
    print(f"  Command state : {'RUN' if server.running else 'STOP'}")
    print("=" * 50)
    print("\nControls (type in this terminal while tracker runs):")
    print("  [Enter]  → toggle RUN / STOP command")
    print("  'q'      → quit everything\n")


def command_loop(server: UDPServer):
    """
    Runs in a background thread.
    Pressing Enter toggles the RUN/STOP broadcast command.
    Typing 'q' + Enter shuts everything down.
    """
    while True:
        try:
            user_input = input().strip().lower()
        except EOFError:
            break

        if user_input == 'q':
            print("\n[Init] Quit command received — shutting down...")
            server.set_command(run=False)
            time.sleep(0.2)          # allow one final STOP packet to broadcast
            server.stop_server()
            sys.exit(0)
        else:
            # Toggle RUN ↔ STOP
            new_state = not server.running
            server.set_command(run=new_state)
            state_str = "RUN" if new_state else "STOP"
            print(f"[Init] Broadcast command switched to: {state_str}")


def main():
    # ── 1. Get user configuration ─────────────────────────────────────────────
    num_robots, robot_ids = get_robot_config()

    # ── 2. Start UDP server ───────────────────────────────────────────────────
    server = UDPServer()
    server.set_command(run=True)       # default: RUN
    server.start(rate_hz=20.0)
    print(f"[Init] UDP server started. Broadcasting at 20 Hz.")

    # ── 3. Print status and controls ──────────────────────────────────────────
    print_status(robot_ids, server)

    # ── 4. Start command listener in background ───────────────────────────────
    cmd_thread = threading.Thread(target=command_loop, args=(server,), daemon=True)
    cmd_thread.start()

    # ── 5. Start camera tracker (blocks until 'q' pressed in video window) ───
    try:
        tracker = CameraTracker(server=server, active_robot_ids=robot_ids)
        tracker.run()
    except RuntimeError as e:
        print(f"[Init] Tracker error: {e}")
    except KeyboardInterrupt:
        print("\n[Init] Keyboard interrupt received.")
    finally:
        print("[Init] Sending STOP and shutting down server...")
        server.set_command(run=False)
        time.sleep(0.3)
        server.stop_server()
        print("[Init] Done.")


if __name__ == "__main__":
    main()
