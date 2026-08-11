"""
initiation.py - Swarm Initialisation Entry Point
Prompts the user for robot configuration, starts the UDP broadcast server,
launches the camera tracker, and provides a manual STOP command interface.

KEY CHANGES:
- No background broadcast loop (packets sent on-demand by tracker)
- Server is started but waits for tracker to send packets immediately after processing
"""

import threading
import time
import sys
from udp import UDPServer
from tracker import CameraTracker


def print_startup_message():
    """Display startup message."""
    print("=" * 50)
    print("  Robot Swarm — Initiation")
    print("=" * 50)
    print("\nAuto-detecting all ArUco markers (4x4_50 dictionary)...")
    print("Waiting for camera feed...\n")


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
    # ── 1. Print startup message ──────────────────────────────────────────────
    print_startup_message()

    # ── 2. Start UDP server (no background broadcast) ─────────────────────────
    server = UDPServer()
    server.set_command(run=False)      # default: STOP (safe state)
    server.start(enable_background_broadcast=False)  # Packets sent on-demand
    print(f"[Init] UDP server started (on-demand packet transmission).\n")

    # ── 3. Start command listener in background ───────────────────────────────
    cmd_thread = threading.Thread(target=command_loop, args=(server,), daemon=True)
    cmd_thread.start()

    # ── 4. Start camera tracker (auto-detects all 4x4_50 markers) ─────────────
    try:
        tracker = CameraTracker(server=server, active_robot_ids=None)  # None = detect all
        print("=" * 50)
        print("  Controls")
        print("=" * 50)
        print("  [Enter]  → toggle RUN / STOP command")
        print("  'q'      → quit everything\n")
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
