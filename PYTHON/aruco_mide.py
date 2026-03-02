import cv2
import numpy as np
from collections import defaultdict

# ================== PARÁMETROS DE CÁMARA ==================
camera_matrix = np.array([
    [1999.17838, 0, 928.811574],
    [0, 2006.06097, 478.347147],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([
    0.0906358995, 0.318998982,
   -0.00288859938, 0.000180882259,
   -2.99885280
], dtype=np.float32)

# ================== CONFIGURACIÓN ==================
marker_length = 0.10  # metros
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
detector_params = cv2.aruco.DetectorParameters()

ids_roles = {0: "Referencia", 1: "Robot", 2: "Objetivo"}

# ================== MODELO 3D DEL MARCADOR ==================
obj_points = np.array([
    [-marker_length/2,  marker_length/2, 0],
    [ marker_length/2,  marker_length/2, 0],
    [ marker_length/2, -marker_length/2, 0],
    [-marker_length/2, -marker_length/2, 0]
], dtype=np.float32)

# ================== FILTRO EMA ==================
alpha = 0.25
tvec_filt = defaultdict(lambda: None)

def ema_filter(marker_id, tvec):
    if tvec_filt[marker_id] is None:
        tvec_filt[marker_id] = tvec
    else:
        tvec_filt[marker_id] = alpha * tvec + (1 - alpha) * tvec_filt[marker_id]
    return tvec_filt[marker_id]

# ================== FUNCIONES ==================
def distance(t1, t2):
    return np.linalg.norm(t1.flatten() - t2.flatten())

# ================== CÁMARA ==================
cap = cv2.VideoCapture(0)

print("Sistema ArUco – Medición estable de distancias")
print("ESC para salir")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=detector_params)

    poses = {}
    centers = {}

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            img_points = corners[i].reshape(-1, 2)

            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                continue

            tvec = ema_filter(marker_id, tvec)
            poses[marker_id] = (rvec, tvec)

            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.04)

            center = np.mean(img_points, axis=0).astype(int)
            centers[marker_id] = tuple(center)

            role = ids_roles.get(marker_id, f"ID {marker_id}")
            cv2.putText(frame, role, (center[0]-20, center[1]-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # ================== DISTANCIAS ==================
        y = 30
        for (i, j) in [(0,1), (0,2), (1,2)]:
            if i in poses and j in poses:
                d = distance(poses[i][1], poses[j][1]) * 100
                cv2.putText(frame,
                            f"{ids_roles[i]} - {ids_roles[j]}: {d:.1f} cm",
                            (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0,255,0),
                            2)
                y += 25

                cv2.line(frame, centers[i], centers[j], (255,0,0), 2)

    cv2.imshow("ArUco – Medición estable", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
