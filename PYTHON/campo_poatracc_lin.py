import cv2
import numpy as np
import socket
import struct
from collections import defaultdict

# =====================================================
# ========== CONFIGURACIÓN UDP (ESP32) =================
# =====================================================
ESP32_IP = "192.168.137.48"
ESP32_PORT = 58625
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =====================================================
# ========== PARÁMETROS MOTORES EMG30 ==================
# =====================================================
W_MAX = 12.0
PWM_MAX = 255

def rad_s_to_pwm(w):
    w = np.clip(w, -W_MAX, W_MAX)
    return int((w / W_MAX) * PWM_MAX)

# =====================================================
# ========== PARÁMETROS DE CÁMARA ======================
# =====================================================
camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([[camera_params[0], 0, camera_params[2]],
                          [0, camera_params[1], camera_params[3]],
                          [0, 0, 1]], dtype=np.float32)

dist_coeffs = np.array([0.0906358995, 0.318998982,
                        -0.00288859938, 0.000180882259, -2.99885280])

marker_length = 0.1

# =====================================================
# ========== PARÁMETROS DEL ROBOT ======================
# =====================================================
r = 0.1
b = 0.1425
d = 0.078
L = b + d

kv = 2
kw = 1.2
zona_prot = 0.15

# =====================================================
# ========== FILTROS TEMPORALES ========================
# =====================================================
alpha = 0.3
tvec_filt = defaultdict(lambda: None)
yaw_filt = defaultdict(lambda: None)

# =====================================================
# ========== FUNCIONES AUXILIARES ======================
# =====================================================
def pose_to_marker0(rvec_i, tvec_i, rvec_0, tvec_0):
    R_i, _ = cv2.Rodrigues(rvec_i)
    R_0, _ = cv2.Rodrigues(rvec_0)
    R_0_i = R_0.T @ R_i
    t_0_i = R_0.T @ (tvec_i - tvec_0)
    return R_0_i, t_0_i

def yaw_from_R(R):
    return np.arctan2(R[1, 0], R[0, 0])

def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))

# =====================================================
# ========== CÁMARA Y ARUCO ============================
# =====================================================
cap = cv2.VideoCapture(0)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)

    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        poses_3d = {}
        centers_2d = {}

        for i, marker_id in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # ===== FILTRO tvec =====
            if tvec_filt[marker_id] is None:
                tvec_filt[marker_id] = tvec
            else:
                tvec_filt[marker_id] = alpha*tvec + (1-alpha)*tvec_filt[marker_id]

            # ===== ORIENTACIÓN =====
            R, _ = cv2.Rodrigues(rvec)
            yaw = yaw_from_R(R)

            if yaw_filt[marker_id] is None:
                yaw_filt[marker_id] = yaw
            else:
                yaw_filt[marker_id] = np.arctan2(
                    alpha*np.sin(yaw) + (1-alpha)*np.sin(yaw_filt[marker_id]),
                    alpha*np.cos(yaw) + (1-alpha)*np.cos(yaw_filt[marker_id])
                )

            poses_3d[marker_id] = (rvec, tvec_filt[marker_id])

            # ===== DIBUJO DEL CUADRADO =====
            c = corners[i][0].astype(int)
            cv2.polylines(frame, [c], True, (0, 255, 0), 2)
            cx, cy = np.mean(c, axis=0).astype(int)
            cv2.putText(frame, f"ID {marker_id}",
                        (cx - 20, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)

            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                              rvec, tvec_filt[marker_id], 0.03)

            imgpt, _ = cv2.projectPoints(
                np.array([[0, 0, 0]], dtype=np.float32),
                rvec, tvec_filt[marker_id],
                camera_matrix, dist_coeffs
            )

            centers_2d[marker_id] = tuple(imgpt[0][0].astype(int))

        if all(k in poses_3d for k in [0, 1, 2]):

            rvec_0, tvec_0 = poses_3d[0]

            R0_r, t0_r = pose_to_marker0(*poses_3d[1], rvec_0, tvec_0)
            R0_o, t0_o = pose_to_marker0(*poses_3d[2], rvec_0, tvec_0)

            xr, yr = float(t0_r[0]), float(t0_r[1])
            xo, yo = float(t0_o[0]), float(t0_o[1])
            theta_r = yaw_filt[1]

            dx, dy = xo - xr, yo - yr
            d_obj = np.hypot(dx, dy)

            vx_w = kv * dx
            vy_w = kv * dy

            theta_ref = np.arctan2(dy, dx)
            w = kw * wrap_angle(theta_ref - theta_r)

            if d_obj < zona_prot:
                vx_w = vy_w = 0.0
                w = 0.0

            vx =  np.cos(theta_r)*vx_w + np.sin(theta_r)*vy_w
            vy = -np.sin(theta_r)*vx_w + np.cos(theta_r)*vy_w

            w1 = (vx/r) - (vy/r) - (L/r)*w
            w2 = (vx/r) + (vy/r) + (L/r)*w
            w3 = (vx/r) + (vy/r) - (L/r)*w
            w4 = (vx/r) - (vy/r) + (L/r)*w

            wmax = max(abs(w1), abs(w2), abs(w3), abs(w4))
            if wmax > W_MAX:
                s = W_MAX / wmax
                w1 *= s; w2 *= s; w3 *= s; w4 *= s

            packet = struct.pack('<hhhh',
                                 rad_s_to_pwm(w1),
                                 rad_s_to_pwm(w2),
                                 rad_s_to_pwm(w3),
                                 rad_s_to_pwm(w4))
            sock.sendto(packet, (ESP32_IP, ESP32_PORT))

            cv2.arrowedLine(frame, centers_2d[0], centers_2d[1],
                            (0, 255, 0), 2)
            cv2.arrowedLine(frame, centers_2d[0], centers_2d[2],
                            (255, 0, 0), 2)
            cv2.arrowedLine(frame, centers_2d[1], centers_2d[2],
                            (0, 0, 255), 2)

    else:
        sock.sendto(struct.pack('<hhhh', 0, 0, 0, 0),
                    (ESP32_IP, ESP32_PORT))

    cv2.imshow("Sistema ArUco – Marco del marcador 0", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
