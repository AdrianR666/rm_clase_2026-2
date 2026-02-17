import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import socket

# ================= UDP =================
UDP_IP = "192.168.137.224"
UDP_PORT = 12345
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================= ArUco =================
MARKER_SIZE_METERS = 0.1
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# ================= Cámara =================
cap = cv.VideoCapture(1, cv.CAP_DSHOW)

camera_params = [1999.17838, 2006.06097, 928.811574, 478.347147]
camera_matrix = np.array([
    [camera_params[0], 0, camera_params[2]],
    [0, camera_params[1], camera_params[3]],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([0.0906, 0.3189, -0.0028, 0.00018, -2.998])

# ================= Robot =================
r_rueda = 0.03       # radio rueda [m]
l_eje = 0.12         # distancia entre ruedas [m]

# ===== Reaching the goal + campo atractivo =====
v_max = 0.25         # [m/s]
w_max = 2.5          # [rad/s]
k_r = 0.15           # radio docking [m]

# ================= Funciones =================
def obtener_matriz_homogenea(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T

def transformar_a_referencia_0(T_obj, T_ref):
    return np.linalg.inv(T_ref) @ T_obj

def mapear_velocidad(val):
    return max(-255, min(255, int(val)))

def enviar_velocidades_udp(vl, vr):
    try:
        data = f"{vl:.2f},{vr:.2f}".encode()
        udp_socket.sendto(data, (UDP_IP, UDP_PORT))
        print(f"Enviado -> vL: {vl:.2f}, vR: {vr:.2f}")
    except Exception as e:
        print(f"Error UDP: {e}")

def diferencial_inverse_kinematics(v, w, r, l):
    vr = (2*v + w*l) / (2*r)
    vl = (2*v - w*l) / (2*r)
    return vl, vr

# ==========================================================
# CONTROL UNIFICADO: CAMPO ATRACTIVO + REACHING THE GOAL
# ==========================================================
def control_reaching_goal(xr, yr, theta_r, xo, yo):
    # Error en marco inercial {0}
    dx = xo - xr
    dy = yo - yr
    d = np.hypot(dx, dy)

    # Dirección al objetivo
    theta_g = np.arctan2(dy, dx)
    theta_e = np.arctan2(
        np.sin(theta_g - theta_r),
        np.cos(theta_g - theta_r)
    )

    # Velocidad lineal (reaching the goal)
    if d > k_r:
        v = v_max
    else:
        v = (v_max / k_r) * d

    # Velocidad angular
    w = w_max * np.sin(theta_e)

    # Saturación
    v = np.clip(v, 0.0, v_max)
    w = np.clip(w, -w_max, w_max)

    return v, w

# ================= Loop principal =================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE_METERS,
            camera_matrix, dist_coeffs
        )

        poses = {}
        for i, mid in enumerate(ids.flatten()):
            poses[mid] = (rvecs[i][0], tvecs[i][0])
            cv.drawFrameAxes(
                frame, camera_matrix, dist_coeffs,
                rvecs[i][0], tvecs[i][0], 0.05
            )

        if all(k in poses for k in [0, 1, 2]):

            # Transformaciones a marco inercial {0}
            T_cam_0 = obtener_matriz_homogenea(*poses[0])
            T_cam_1 = obtener_matriz_homogenea(*poses[1])
            T_cam_2 = obtener_matriz_homogenea(*poses[2])

            T_0_1 = transformar_a_referencia_0(T_cam_1, T_cam_0)
            T_0_2 = transformar_a_referencia_0(T_cam_2, T_cam_0)

            # Posiciones en {0}
            xr, yr = T_0_1[0, 3], T_0_1[1, 3]
            xo, yo = T_0_2[0, 3], T_0_2[1, 3]

            # Orientación del robot (yaw en {0})
            theta_r = np.arctan2(T_0_1[1, 0], T_0_1[0, 0])

            # ===== CONTROL =====
            v, w = control_reaching_goal(xr, yr, theta_r, xo, yo)

            # Cinemática diferencial
            vl, vr = diferencial_inverse_kinematics(
                v, w, r_rueda, l_eje
            )

            enviar_velocidades_udp(
                mapear_velocidad(vl),
                mapear_velocidad(vr)
            )

        else:
            enviar_velocidades_udp(0, 0)
            print("⚠️ Faltan marcadores")

    else:
        enviar_velocidades_udp(0, 0)
        print("⚠️ Sin detección")

    cv.imshow("Control Reaching the Goal – Marco ArUco 0", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
