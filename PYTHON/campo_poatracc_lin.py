import cv2 
import numpy as np
import socket
import struct

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

# =====================================================
marker_length = 0.1

# =====================================================
# ========== PARÁMETROS DEL ROBOT ======================
# =====================================================
r = 0.1
b = 0.1425
d = 0.078

kv = 2
kw = 1.2                 # 🔧 ACTIVADO (antes era 0)
ap = np.deg2rad(2)
zona_prot = 0.15

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
            rvec = rvecs[i]
            tvec = tvecs[i].flatten()

            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            imgpt, _ = cv2.projectPoints(
                np.array([[0, 0, 0]], dtype=np.float32),
                rvec, tvec, camera_matrix, dist_coeffs
            )

            centers_2d[marker_id] = tuple(imgpt[0][0].astype(int))
            poses_3d[marker_id] = (rvec, tvec)

        if all(k in poses_3d for k in [0, 1, 2]):

            rvec_0, tvec_0 = poses_3d[0]

            R0_r, t0_r = pose_to_marker0(*poses_3d[1], rvec_0, tvec_0)
            R0_o, t0_o = pose_to_marker0(*poses_3d[2], rvec_0, tvec_0)

            xr, yr = t0_r[0], t0_r[1]
            xo, yo = t0_o[0], t0_o[1]

            theta_r = yaw_from_R(R0_r)

            # ================= CONTROL =================
            dx, dy = xo - xr, yo - yr
            d_obj = np.hypot(dx, dy)

            # -------- VELOCIDADES EN MUNDO --------
            vx_w = kv * dx
            vy_w = kv * dy

            theta_ref = np.arctan2(dy, dx)
            da = wrap_angle(theta_ref - theta_r)
            w = kw * da           # 🔧 CONTROL ANGULAR CONTINUO

            if d_obj < zona_prot:
                vx_w = vy_w = 0.0
                w = 0.0

            # ======= TRANSFORMACIÓN MUNDO → ROBOT =======
            vx =  np.cos(theta_r)*vx_w + np.sin(theta_r)*vy_w
            vy = -np.sin(theta_r)*vx_w + np.cos(theta_r)*vy_w

            # ================= CINEMÁTICA INVERSA =================
            L = b + d

            w1 = (vx/r) - (vy/r) - (L/r)*w
            w2 = (vx/r) + (vy/r) + (L/r)*w
            w3 = (vx/r) + (vy/r) - (L/r)*w
            w4 = (vx/r) - (vy/r) + (L/r)*w

            # ======= SATURACIÓN VECTORIAL =======
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

            # ================= VECTORES =================
            cv2.arrowedLine(frame, centers_2d[0], centers_2d[1],
                            (0, 255, 0), 2, tipLength=0.2)
            cv2.arrowedLine(frame, centers_2d[0], centers_2d[2],
                            (255, 0, 0), 2, tipLength=0.2)
            cv2.arrowedLine(frame, centers_2d[1], centers_2d[2],
                            (0, 0, 255), 2, tipLength=0.2)

    else:
        sock.sendto(struct.pack('<hhhh', 0, 0, 0, 0),
                    (ESP32_IP, ESP32_PORT))

    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("Sistema ArUco – Marco del marcador 0", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
