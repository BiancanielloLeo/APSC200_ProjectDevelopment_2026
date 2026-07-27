"""
Code Authored by Keegan Kelly
Modified to add white border, cut line, and console marker selection
"""
import cv2
import numpy as np
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
print("Saving to:", os.getcwd())

TOTAL_SIZE  = 375
BORDER_PX   = 19
MARKER_SIZE = TOTAL_SIZE - 2 * BORDER_PX  # 337px

# --- Ask for marker ID ---
marker_id = int(input("Enter marker ID (0-49): "))
if marker_id < 0 or marker_id > 49:
    print("Invalid ID for DICT_4X4_50. Must be between 0 and 49.")
    exit()

# --- Generate marker ---
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
tag = np.zeros((MARKER_SIZE, MARKER_SIZE, 1), dtype='uint8')
cv2.aruco.generateImageMarker(arucoDict, marker_id, MARKER_SIZE, tag, 1)

# --- Place marker on white canvas ---
canvas = np.ones((TOTAL_SIZE, TOTAL_SIZE), dtype='uint8') * 255
canvas[BORDER_PX:BORDER_PX + MARKER_SIZE, BORDER_PX:BORDER_PX + MARKER_SIZE] = tag[:, :, 0]

# --- Draw dotted cut line ---
DOT_LEN   = 6
GAP_LEN   = 6
CUT_COLOR = 128

def draw_dotted_line(img, pt1, pt2, color, dot_len, gap_len):
    x1, y1 = pt1
    x2, y2 = pt2
    dx = x2 - x1
    dy = y2 - y1
    length = np.hypot(dx, dy)
    step = dot_len + gap_len
    steps = int(length / step)
    for i in range(steps + 1):
        t_start = i * step / length
        t_end   = min((i * step + dot_len) / length, 1.0)
        sx = int(x1 + t_start * dx)
        sy = int(y1 + t_start * dy)
        ex = int(x1 + t_end   * dx)
        ey = int(y1 + t_end   * dy)
        cv2.line(img, (sx, sy), (ex, ey), color, 1)

inset = 1
draw_dotted_line(canvas, (inset, inset),                          (TOTAL_SIZE - inset, inset),                  CUT_COLOR, DOT_LEN, GAP_LEN)
draw_dotted_line(canvas, (TOTAL_SIZE - inset, inset),             (TOTAL_SIZE - inset, TOTAL_SIZE - inset),     CUT_COLOR, DOT_LEN, GAP_LEN)
draw_dotted_line(canvas, (TOTAL_SIZE - inset, TOTAL_SIZE - inset),(inset, TOTAL_SIZE - inset),                  CUT_COLOR, DOT_LEN, GAP_LEN)
draw_dotted_line(canvas, (inset, TOTAL_SIZE - inset),             (inset, inset),                               CUT_COLOR, DOT_LEN, GAP_LEN)

# --- Save to "4x4_50 ArUcos" folder ---
output_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "4x4_50 ArUcos")
os.makedirs(output_folder, exist_ok=True)

filename = os.path.join(output_folder, f"aruco{marker_id}.png")

cv2.imwrite(filename, canvas)
print(f"Saved {filename} — marker: {MARKER_SIZE}px, border: {BORDER_PX}px, total: {TOTAL_SIZE}px")
