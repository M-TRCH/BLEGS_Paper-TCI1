"""
AR-Tag-5Bar-Linkage.py
======================
ประมวลผลไฟล์วิดีโอเพื่อตรวจจับ ArUco Markers (ID 1-4) และวาด
Separated Five-Bar Linkage ซ้อนทับ คำนวณตำแหน่งปลายขา (End-Effector E)
ใน Pixel Space โดยใช้อัตราส่วนความยาวลิงก์ที่ Calibrate แล้ว

Marker - จุดทาง Kinematic:
    ID 1 = A  (จุดมอเตอร์ A)
    ID 2 = B  (จุดมอเตอร์ B)
    ID 3 = C  (ปลายลิงก์ AC -- 105 mm)
    ID 4 = D  (ปลายลิงก์ BD -- 105 mm)
    E = คำนวณใน Pixel Space (Circle Intersection C(145) กับ D(145))
        รัศมี Calibrate จากระยะ AC/BD ที่วัดได้ (อัตราส่วน 145/105)

ควบคุม:
    [Space]  หยุด / เล่น
    [->] / d กระโดดไปข้างหน้า 5 วินาที
    [<-] / a กระโดดถอยหลัง 5 วินาที
    [q]      ออก

ที่มา: https://github.com/M-TRCH/BLEGS_Actuator-Unit/tree/migrate-python-files/python/vision
"""

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import time
import sys
import os
import threading
import collections
import csv
import tkinter as tk
from tkinter import filedialog

# -----------------------------------------------------------------
# 1. ตั้งค่า ArUco Detector
# -----------------------------------------------------------------
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()

aruco_params.adaptiveThreshWinSizeMin    = 3
aruco_params.adaptiveThreshWinSizeMax    = 23
aruco_params.adaptiveThreshWinSizeStep   = 10
aruco_params.minMarkerPerimeterRate      = 0.03
aruco_params.maxMarkerPerimeterRate      = 4.0
aruco_params.polygonalApproxAccuracyRate = 0.05
aruco_params.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_NONE

detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# -----------------------------------------------------------------
# 2. ขนาด Marker และ Object Points
# -----------------------------------------------------------------
MARKER_SIZE_METERS = 0.03125   # 3.125 cm

OBJ_POINTS = np.array([
    [-MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
    [-MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
], dtype=np.float32)

# -----------------------------------------------------------------
# 3. พารามิเตอร์ Separated Five-Bar Linkage (หน่วย mm)
# -----------------------------------------------------------------
L_AC = 105.0
L_BD = 105.0
L_CE = 145.0
L_DE = 145.0

ID_TO_POINT = {1: 'A', 2: 'B', 3: 'C', 4: 'D'}

# -----------------------------------------------------------------
# Performance Flags
# -----------------------------------------------------------------
RUN_PNP         = False
DETECT_INTERVAL = 1

# -----------------------------------------------------------------
# 4. สี / สไตล์ Overlay (BGR)
# -----------------------------------------------------------------
COLOR_A_B  = (180, 180, 180)
COLOR_A_C  = (0,   255, 255)
COLOR_B_D  = (255, 255,   0)
COLOR_C_E  = (0,   165, 255)
COLOR_D_E  = (255,   0, 255)

POINT_COLOR = {
    'A': (0,   255, 255),
    'B': (255, 255,   0),
    'C': (0,   165, 255),
    'D': (255,   0, 255),
    'E': (0,   255,   0),
}
LINK_THICK  = 2
DOT_RADIUS  = 5

# -----------------------------------------------------------------
# 5. โหลด Camera Calibration
# -----------------------------------------------------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
calib_file = os.path.join(script_dir,
    "camera_calibration", "camera_params_sigma35f14dgdn_1080p60.npz")

try:
    calib_data    = np.load(calib_file)
    camera_matrix = calib_data["mtx"]
    print(f"Loaded camera parameters from: {calib_file}")
except FileNotFoundError:
    print(f"Calibration file not found: {calib_file}")
    print("   Using default 1920x1080")
    camera_matrix = np.array([
        [1400., 0., 960.],
        [0., 1400., 540.],
        [0.,    0.,   1.],
    ], dtype=np.float64)

dist_coeffs = np.zeros((5, 1), dtype=np.float64)

# -----------------------------------------------------------------
# 6. คำนวณ E ใน Pixel Space (2D)
#
#    อัลกอริทึม (mirror calculate_fk_no_ef):
#    1. วัด pixel-distance ของ AC และ BD (= 105 mm จริง) -> ได้ scale
#    2. r_CE = d_AC_px * (145/105)   r_DE = d_BD_px * (145/105)
#    3. circle-intersection C(r_CE) กับ D(r_DE) ใน pixel space
#    4. เลือก E ที่อยู่ฝั่งปลายขา
# -----------------------------------------------------------------
def find_end_effector_px(A_px, B_px, C_px, D_px):
    """คืน (u, v) ของ E ใน pixel หรือ None"""
    A = np.array(A_px, dtype=np.float64)
    B = np.array(B_px, dtype=np.float64)
    C = np.array(C_px, dtype=np.float64)
    D = np.array(D_px, dtype=np.float64)

    d_AC = np.linalg.norm(C - A)
    d_BD = np.linalg.norm(D - B)
    if d_AC < 1e-6 or d_BD < 1e-6:
        return None

    r_CE = d_AC * (L_CE / L_AC)
    r_DE = d_BD * (L_DE / L_BD)

    V_CD = D - C
    d    = np.linalg.norm(V_CD)
    if d < 1e-6 or d > r_CE + r_DE or d < abs(r_CE - r_DE):
        return None

    a    = (r_CE**2 - r_DE**2 + d**2) / (2.0 * d)
    h    = np.sqrt(max(0.0, r_CE**2 - a**2))
    u_cd   = V_CD / d
    v_perp = np.array([-u_cd[1], u_cd[0]])

    E1 = C + a * u_cd + h * v_perp
    E2 = C + a * u_cd - h * v_perp

    mid_AB = (A + B) * 0.5
    y_axis = (C + D) * 0.5 - mid_AB
    y_len  = np.linalg.norm(y_axis)
    if y_len < 1e-6:
        E = E1
    else:
        y_axis /= y_len
        E = E1 if np.dot(E1 - mid_AB, y_axis) >= np.dot(E2 - mid_AB, y_axis) else E2

    return (int(round(E[0])), int(round(E[1])))


def draw_linkage(img, pts_px):
    """วาด Separated Five-Bar Linkage ลงบนภาพ (in-place)"""
    h_img, w_img = img.shape[:2]

    def safe(key):
        p = pts_px.get(key)
        if p is None:
            return None
        if 0 <= p[0] < w_img and 0 <= p[1] < h_img:
            return p
        return None

    def line(k1, k2, color, thick=LINK_THICK):
        p1, p2 = safe(k1), safe(k2)
        if p1 and p2:
            cv2.line(img, p1, p2, color, thick, cv2.LINE_AA)

    def dot(key, color, r=DOT_RADIUS):
        p = safe(key)
        if p:
            cv2.circle(img, p, r, (0, 0, 0), -1)
            cv2.circle(img, p, r - 1, color, -1)

    def label(key, color, text=None):
        p = safe(key)
        if p:
            lbl = text if text else key
            cv2.putText(img, lbl, (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    line('A', 'B', COLOR_A_B, 1)
    line('A', 'C', COLOR_A_C)
    line('B', 'D', COLOR_B_D)
    line('C', 'E', COLOR_C_E, LINK_THICK + 1)
    line('D', 'E', COLOR_D_E, LINK_THICK + 1)

    for name in ('A', 'B', 'C', 'D'):
        dot(name, POINT_COLOR[name])
        label(name, POINT_COLOR[name])

    epos = safe('E')
    if epos:
        cv2.circle(img, epos, DOT_RADIUS + 4, POINT_COLOR['E'], 2, cv2.LINE_AA)
        cv2.circle(img, epos, DOT_RADIUS + 1, POINT_COLOR['E'], -1)
        cv2.putText(img, 'E', (epos[0] + 8, epos[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, POINT_COLOR['E'], 2, cv2.LINE_AA)


# -----------------------------------------------------------------
# 7. เลือกไฟล์วิดีโอ
# -----------------------------------------------------------------
if len(sys.argv) > 1:
    video_path = sys.argv[1]
else:
    root = tk.Tk()
    root.withdraw()
    video_path = filedialog.askopenfilename(
        title="เลือกไฟล์วิดีโอ",
        filetypes=[
            ("Video files", "*.mp4 *.avi *.mov *.mkv *.wmv *.flv *.webm"),
            ("All files", "*.*"),
        ],
    )
    root.destroy()

if not video_path or not os.path.isfile(video_path):
    print("No file selected or file not found -- exiting")
    sys.exit(1)

print(f"\nVideo file: {video_path}")

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("ไม่สามารถเปิดไฟล์วิดีโอได้")
    sys.exit(1)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
video_fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
vid_w        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
vid_h        = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Resolution: {vid_w}x{vid_h}  |  FPS: {video_fps:.1f}  |  Frames: {total_frames}")

# -----------------------------------------------------------------
# CSV Log Setup
# -----------------------------------------------------------------
_log_stamp  = time.strftime("%Y%m%d_%H%M%S")
_log_name   = os.path.splitext(os.path.basename(video_path))[0]
_log_path   = os.path.join(os.path.dirname(video_path),
                           f"{_log_name}_linkage_{_log_stamp}.csv")
_log_file   = open(_log_path, "w", newline="", encoding="utf-8")
_log_writer = csv.writer(_log_file)
_log_writer.writerow(["timestamp_s", "frame", "E_x_mm", "E_y_mm"])
print(f"Logging to: {_log_path}")

DISPLAY_SCALE   = min(1280 / vid_w, 720 / vid_h, 1.0)
disp_w          = int(vid_w * DISPLAY_SCALE)
disp_h          = int(vid_h * DISPLAY_SCALE)
DETECTION_SCALE = 0.5
frame_period    = 1.0 / video_fps

# -----------------------------------------------------------------
# 8. แปลง Pixel -> Robot mm Frame
#    A_robot = (-42.5, 0)   B_robot = (42.5, 0)  (mm)
# -----------------------------------------------------------------
A_ROBOT = np.array([-42.5, 0.0])
B_ROBOT = np.array([ 42.5, 0.0])

def px_to_robot(P_px, A_px, B_px):
    """แปลง pixel-coordinate -> robot mm frame"""
    A  = np.array(A_px, dtype=np.float64)
    B  = np.array(B_px, dtype=np.float64)
    P  = np.array(P_px, dtype=np.float64)

    mid   = (A + B) * 0.5
    AB    = B - A
    d_AB  = np.linalg.norm(AB)
    if d_AB < 1e-6:
        return None

    x_hat = AB / d_AB
    y_hat = np.array([x_hat[1], -x_hat[0]])
    scale = d_AB / 85.0

    v = P - mid
    rx = np.dot(v, x_hat) / scale
    ry = np.dot(v, y_hat) / scale
    return np.array([rx, ry])


# -----------------------------------------------------------------
# 9. ตั้งค่า Matplotlib -- Live 2D Linkage Plot (Robot mm Frame)
# -----------------------------------------------------------------
plt.ion()
fig_rob, ax_rob = plt.subplots(figsize=(6, 8))
fig_rob.suptitle("5-Bar Linkage -- Robot Frame (mm)", fontsize=12)
ax_rob.set_xlabel("X (mm)")
ax_rob.set_ylabel("Y (mm)")
ax_rob.set_xlim(-200, 200)
ax_rob.set_ylim(-280, 80)
ax_rob.set_aspect("equal")
ax_rob.grid(True, alpha=0.35)
ax_rob.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax_rob.axvline(0, color="gray", linewidth=0.8, linestyle="--")

ax_rob.plot(*A_ROBOT, marker="o", markersize=10,
            color=(0, 1, 1), label="A (-42.5, 0)")
ax_rob.plot(*B_ROBOT, marker="o", markersize=10,
            color=(1, 1, 0), label="B (42.5, 0)")
ax_rob.annotate("A", xy=A_ROBOT, xytext=(-8, 6), textcoords="offset points",
                color=(0, 1, 1), fontsize=10, fontweight="bold")
ax_rob.annotate("B", xy=B_ROBOT, xytext=(4, 6), textcoords="offset points",
                color=(1, 1, 0), fontsize=10, fontweight="bold")

_MCOLOR = {"C": (1, 0.65, 0), "D": (1, 0, 1), "E": (0, 1, 0)}
_rob_pts  = {}
_rob_lbls = {}
_rob_segs = {}

for name in ("C", "D", "E"):
    sc, = ax_rob.plot([], [], marker="o", markersize=7,
                      color=_MCOLOR[name], linestyle="None", label=name)
    lb = ax_rob.annotate("", xy=(0, 0), xytext=(5, 5),
                         textcoords="offset points",
                         color=_MCOLOR[name], fontsize=9)
    _rob_pts[name]  = sc
    _rob_lbls[name] = lb

for (k1, k2, c) in [("A", "B",  (0.7, 0.7, 0.7)),
                     ("A", "C",  (0, 1, 1)),
                     ("B", "D",  (1, 1, 0)),
                     ("C", "E",  (1, 0.65, 0)),
                     ("D", "E",  (1, 0, 1))]:
    seg, = ax_rob.plot([], [], color=c, linewidth=2)
    _rob_segs[(k1, k2)] = seg

ax_rob.legend(loc="upper right", fontsize=8)
fig_rob.tight_layout()
fig_rob.canvas.draw()
plt.pause(0.001)

_last_plot_t   = time.time()
_PLOT_INTERVAL = 0.15

fig_rob.canvas.draw()
_blit_bg = fig_rob.canvas.copy_from_bbox(ax_rob.bbox)
_blit_artists = (list(_rob_pts.values())
                 + list(_rob_lbls.values())
                 + list(_rob_segs.values()))


def update_robot_plot(rob_pts):
    """อัปเดตกราฟ robot mm frame"""
    global _last_plot_t
    now = time.time()
    if now - _last_plot_t < _PLOT_INTERVAL:
        return
    _last_plot_t = now

    all_pts = {"A": A_ROBOT, "B": B_ROBOT}
    all_pts.update(rob_pts)

    for name in ("C", "D", "E"):
        if name in rob_pts:
            xy = rob_pts[name]
            _rob_pts[name].set_data([xy[0]], [xy[1]])
            _rob_lbls[name].set_position(xy)
            _rob_lbls[name].set_text(f"{name} ({xy[0]:.1f},{xy[1]:.1f})")
        else:
            _rob_pts[name].set_data([], [])
            _rob_lbls[name].set_text("")

    for (k1, k2), seg in _rob_segs.items():
        if k1 in all_pts and k2 in all_pts:
            p1, p2 = all_pts[k1], all_pts[k2]
            seg.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        else:
            seg.set_data([], [])

    fig_rob.canvas.restore_region(_blit_bg)
    for art in _blit_artists:
        ax_rob.draw_artist(art)
    fig_rob.canvas.blit(ax_rob.bbox)
    fig_rob.canvas.flush_events()


# -----------------------------------------------------------------
# Threaded Video Reader
# -----------------------------------------------------------------
_frame_q    = collections.deque(maxlen=8)
_reader_run = True

def _reader_thread(cap_obj):
    global _reader_run
    while _reader_run:
        pos_ms = cap_obj.get(cv2.CAP_PROP_POS_MSEC)
        ret, frm = cap_obj.read()
        if not ret:
            _frame_q.append(None)
            break
        _frame_q.append((frm, pos_ms))

_reader_t = threading.Thread(target=_reader_thread, args=(cap,), daemon=True)
_reader_t.start()

# -----------------------------------------------------------------
# 10. Loop หลัก
# -----------------------------------------------------------------
frame_idx     = 0
paused        = False
seek_step     = int(video_fps * 5)
frame         = None
last_pts_px   = {}
detect_ctr    = 0

_fps_t0       = time.perf_counter()
_fps_cnt      = 0
_fps_display  = 0.0
_play_t0      = None

print("\n[Space] Pause/Play  |  [->] Forward 5s  |  [<-] Back 5s  |  [q] Quit\n")

while True:
    if not paused:
        new_item = None
        while _frame_q:
            new_item = _frame_q.popleft()
        if new_item is None and not _reader_t.is_alive():
            print("End of video")
            break
        if new_item is not None:
            frame, _pos_ms = new_item
            timestamp  = _pos_ms / 1000.0
            frame_idx  = int(round(_pos_ms * video_fps / 1000.0))
            if _play_t0 is None:
                _play_t0 = time.perf_counter()

    if frame is None:
        cv2.waitKey(1)
        continue

    detect_ctr += 1
    run_detect = (detect_ctr % DETECT_INTERVAL == 0)

    pts_px   = {}
    tvec_3d  = {}

    if run_detect:
        if DETECTION_SCALE < 1.0:
            small = cv2.resize(frame, None, fx=DETECTION_SCALE,
                               fy=DETECTION_SCALE,
                               interpolation=cv2.INTER_LINEAR)
            gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        else:
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)
        if DETECTION_SCALE < 1.0 and corners:
            corners = [c / DETECTION_SCALE for c in corners]

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, corner in enumerate(corners):
                mid_id = int(ids[i][0])
                if mid_id not in ID_TO_POINT:
                    continue
                pt_name = ID_TO_POINT[mid_id]
                center  = corner[0].mean(axis=0).astype(np.int32)
                pts_px[pt_name] = tuple(center)

                if RUN_PNP:
                    ok, rvec, tvec = cv2.solvePnP(
                        OBJ_POINTS, corner[0],
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if ok:
                        tvec_3d[pt_name] = tvec.ravel()
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                          rvec, tvec, 0.02)

    # คำนวณ E
    E_px = None
    if all(k in pts_px for k in ('A', 'B', 'C', 'D')):
        E_px = find_end_effector_px(
            pts_px['A'], pts_px['B'],
            pts_px['C'], pts_px['D'],
        )
        if E_px:
            pts_px['E'] = E_px

    # แปลง pixel -> robot mm
    rob_pts = {}
    if 'A' in pts_px and 'B' in pts_px:
        for name in ('C', 'D', 'E'):
            if name in pts_px:
                r = px_to_robot(pts_px[name], pts_px['A'], pts_px['B'])
                if r is not None:
                    rob_pts[name] = r
        update_robot_plot(rob_pts)
        if 'E' in rob_pts:
            ex, ey = rob_pts['E']
            _log_writer.writerow([f"{timestamp:.4f}", frame_idx,
                                  f"{ex:.3f}", f"{ey:.3f}"])

    if pts_px:
        last_pts_px = pts_px
    draw_linkage(frame, pts_px if pts_px else last_pts_px)

    # HUD
    ts_text = f"t={timestamp:.2f}s  f={frame_idx}/{total_frames}"
    cv2.rectangle(frame, (5, 5), (390, 32), (0, 0, 0), -1)
    cv2.putText(frame, ts_text, (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

    _fps_cnt += 1
    _fps_now  = time.perf_counter()
    if _fps_now - _fps_t0 >= 1.0:
        _fps_display = _fps_cnt / (_fps_now - _fps_t0)
        _fps_cnt     = 0
        _fps_t0      = _fps_now
    cv2.putText(frame, f"FPS: {_fps_display:.0f}",
                (vid_w - 160, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0), 2, cv2.LINE_AA)

    if paused:
        cv2.putText(frame, "PAUSED", (vid_w // 2 - 90, vid_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3, cv2.LINE_AA)

    display = cv2.resize(frame, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("5-Bar Linkage -- AR Tag  (Space / a / d / q)", display)

    # Real-time sync
    if paused:
        wait_ms = 0
    elif _play_t0 is not None:
        elapsed   = time.perf_counter() - _play_t0
        ideal     = frame_idx * frame_period
        remaining = ideal - elapsed
        wait_ms   = max(1, int(remaining * 1000))
    else:
        wait_ms = 1

    key = cv2.waitKey(wait_ms) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
        if not paused:
            _play_t0 = time.perf_counter() - frame_idx * frame_period
    elif key in (83, ord('d')):
        _reader_run = False
        _reader_t.join(timeout=0.5)
        frame_idx = min(frame_idx + seek_step, total_frames - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        _frame_q.clear()
        ret, frame = cap.read()
        if not ret:
            break
        _reader_run = True
        _reader_t = threading.Thread(target=_reader_thread, args=(cap,), daemon=True)
        _reader_t.start()
        _play_t0 = time.perf_counter() - frame_idx * frame_period
    elif key in (81, ord('a')):
        _reader_run = False
        _reader_t.join(timeout=0.5)
        frame_idx = max(frame_idx - seek_step, 0)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        _frame_q.clear()
        ret, frame = cap.read()
        if not ret:
            break
        _reader_run = True
        _reader_t = threading.Thread(target=_reader_thread, args=(cap,), daemon=True)
        _reader_t.start()
        _play_t0 = time.perf_counter() - frame_idx * frame_period

# -----------------------------------------------------------------
# 11. ปิด
# -----------------------------------------------------------------
_reader_run = False
_reader_t.join(timeout=1.0)
cap.release()
cv2.destroyAllWindows()
_log_file.close()
print(f"Log saved: {_log_path}")
plt.ioff()
plt.show()
print("Exiting program")
