"""
find_exposure.py - Interactive exposure finder for the C922
Type a number and press Enter to try it. Press 'q' Enter to quit.
Typical indoor overhead range is -4 to -8. More negative = darker.
"""

import cv2
import platform
import threading

CAMERA_INDEX = 1
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

if platform.system() == "Windows":
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
else:
    cap = cv2.VideoCapture(CAMERA_INDEX)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
cap.set(cv2.CAP_PROP_AUTOFOCUS,    0)
cap.set(cv2.CAP_PROP_FOCUS,        0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

exposure = -6
cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

print("=" * 50)
print("  Exposure Finder")
print("=" * 50)
print("Type a number and press Enter to apply it.")
print("Try values like -4, -5, -6, -7, -8")
print("More negative = darker = less glare")
print("Type 'q' and press Enter to quit")
print(f"Starting at: {exposure}")
print("=" * 50)

quit_flag = threading.Event()

def input_loop():
    global exposure
    while not quit_flag.is_set():
        try:
            val = input("Exposure value: ").strip()
        except EOFError:
            break
        if val.lower() == 'q':
            quit_flag.set()
            break
        try:
            exposure = int(val)
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
            print(f"  → Applied exposure {exposure}")
            print(f"  → Set CAMERA_EXPOSURE = {exposure} in tracker.py")
        except ValueError:
            print("  Enter a whole number (e.g. -6) or 'q' to quit")

t = threading.Thread(target=input_loop, daemon=True)
t.start()

cv2.namedWindow("Exposure Finder", cv2.WINDOW_NORMAL)

while not quit_flag.is_set():
    ret, frame = cap.read()
    if not ret:
        break

    cv2.putText(frame, f"Exposure: {exposure}", (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
    cv2.putText(frame, "Type value + Enter in terminal to change",
                (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.imshow("Exposure Finder", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        quit_flag.set()
        break

cap.release()
cv2.destroyAllWindows()
print(f"\nFinal exposure value: {exposure}")
print(f"Set CAMERA_EXPOSURE = {exposure} in tracker.py")
