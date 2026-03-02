import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import socket
from collections import defaultdict

# ================= UDP =================
UDP_IP = "192.168.137.115"
UDP_PORT = 12345
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================= ArUco =================
MARKER_SIZE_METERS = 0.1
MARKER_IDS = [0, 1, 2]  # 0 = referencia, 1 = robot, 2 = objetivo
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ================= Cámara =================
cap = cv.VideoCapture(0, cv.CAP_DSHOW)

camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([
    [camera_params[0], 0, camera_params[2]],
    [0, camera_params[1], camera_params[3]],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([0.0906, 0.3189, -0.0028, 0.00018, -2.998])

# ================= Robot =================
r_rueda = 0.03
l_eje = 0.12

# ================= Control =================
kv = 10
kw = 3.0
v_max = 3
w_max = 1
k_r = 0.15

# ================= FILTROS =================
alpha = 0.3
tvec_filt = defaultdict(lambda: None)
yaw_filt = defaultdict(lambda: None)

# ================= Funciones =================
def obtener_matriz_homogenea(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T

def transformar_a_referencia_0(T_obj, T_ref):
    return np.linalg.inv(T_ref) @ T_obj

def enviar_velocidades_udp(vl, vr):
    data = f"{vl:.2f},{vr:.2f}".encode()
    udp_socket.sendto(data, (UDP_IP, UDP_PORT))

def diferencial_inverse_kinematics(v, w, r, l):
    vr = (2*v + w*l) / (2*r)
    vl = (2*v - w*l) / (2*r)
    return vl, vr

def control_potencial_reaching(xr, yr, theta_r, xo, yo):
    dx = xo - xr
    dy = yo - yr
    d = np.hypot(dx, dy)

    theta_g = np.arctan2(dy, dx)
    theta_e = np.arctan2(np.sin(theta_g-theta_r), np.cos(theta_g-theta_r))

    v = kv * min(d, k_r)
    w = kw * np.sin(theta_e)

    v = np.clip(v, 0, v_max)
    w = np.clip(w, -w_max, w_max)

    return v, w

# ================= Loop principal =================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE_METERS, camera_matrix, dist_coeffs
        )

        poses = {}

        for i, mid in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # ===== FILTRO EMA tvec =====
            if tvec_filt[mid] is None:
                tvec_filt[mid] = tvec
            else:
                tvec_filt[mid] = alpha*tvec + (1-alpha)*tvec_filt[mid]

            # ===== ORIENTACIÓN =====
            R, _ = cv.Rodrigues(rvec)
            yaw = np.arctan2(R[1,0], R[0,0])

            if yaw_filt[mid] is None:
                yaw_filt[mid] = yaw
            else:
                yaw_filt[mid] = np.arctan2(
                    alpha*np.sin(yaw) + (1-alpha)*np.sin(yaw_filt[mid]),
                    alpha*np.cos(yaw) + (1-alpha)*np.cos(yaw_filt[mid])
                )

            poses[mid] = (rvec, tvec_filt[mid])

            # ===== DIBUJO =====
            c = corners[i][0].astype(int)
            cv.polylines(frame, [c], True, (0,255,0), 2)
            cx, cy = np.mean(c, axis=0).astype(int)
            cv.putText(frame, f"ID {mid}", (cx-20, cy-10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            cv.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                             rvec, tvec_filt[mid], 0.05)

        if all(k in poses for k in [0,1,2]):
            T0 = obtener_matriz_homogenea(*poses[0])
            T1 = transformar_a_referencia_0(obtener_matriz_homogenea(*poses[1]), T0)
            T2 = transformar_a_referencia_0(obtener_matriz_homogenea(*poses[2]), T0)

            xr, yr = float(T1[0,3]), float(T1[1,3])
            xo, yo = float(T2[0,3]), float(T2[1,3])
            theta_r = yaw_filt[1]

            v, w = control_potencial_reaching(xr, yr, theta_r, xo, yo)
            vl, vr = diferencial_inverse_kinematics(v, w, r_rueda, l_eje)
            enviar_velocidades_udp(vl, vr)
        else:
            enviar_velocidades_udp(0, 0)

    else:
        enviar_velocidades_udp(0, 0)

    cv.imshow("Campos Potenciales + ArUco", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
